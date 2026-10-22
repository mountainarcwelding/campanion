//! STL read and write.
//!
//! Binary is little-endian: an 80-byte header, a `u32` triangle count, then 50 bytes per
//! triangle (3×f32 normal, 3×3×f32 vertices, one `u16` attribute). The stored normal and the
//! attribute are ignored. ASCII repeats `solid` / `facet` / `endsolid`. Keywords are
//! case-insensitive and may be separated by any ASCII whitespace. A file is binary when its
//! length is exactly `84 + 50 * n` for the count at offset 80. That test wins over a header
//! that begins with `solid`, which some binary files do.

use crate::{Mesh, Result, Triangle, Vec3, check_triangle_count, invalid, parse_at, parse_error};

/// Read a binary or ASCII STL from bytes.
pub fn parse_stl(bytes: &[u8]) -> Result<Mesh> {
    if let Some(count) = binary_triangle_count(bytes) {
        parse_binary(bytes, count)
    } else if starts_with_solid(bytes) {
        parse_ascii(bytes)
    } else {
        Err(parse_error("not an STL file: byte length does not match a binary STL and the text does not start with 'solid'"))
    }
}

/// Write a binary STL. Vertices are stored as little-endian `f32`.
pub fn write_stl_binary(mesh: &Mesh) -> Vec<u8> {
    // `Mesh::new` rejects more than `MAX_TRIANGLES`, which fits in `u32`.
    let Ok(count) = u32::try_from(mesh.triangles().len()) else {
        return Vec::new();
    };
    let mut out = Vec::new();
    let mut header = [0u8; 80];
    for (slot, byte) in header.iter_mut().zip(b"campanion-mesh binary stl".iter()) {
        *slot = *byte;
    }
    out.extend_from_slice(&header);
    out.extend_from_slice(&count.to_le_bytes());
    for triangle in mesh.triangles() {
        let normal = triangle.normal().unwrap_or(Vec3::new(0.0, 0.0, 0.0));
        extend_f32(&mut out, normal.x);
        extend_f32(&mut out, normal.y);
        extend_f32(&mut out, normal.z);
        for vertex in [triangle.a, triangle.b, triangle.c] {
            extend_f32(&mut out, vertex.x);
            extend_f32(&mut out, vertex.y);
            extend_f32(&mut out, vertex.z);
        }
        out.extend_from_slice(&0u16.to_le_bytes());
    }
    out
}

fn extend_f32(out: &mut Vec<u8>, value: f64) {
    out.extend_from_slice(&(value as f32).to_le_bytes());
}

fn binary_triangle_count(bytes: &[u8]) -> Option<u32> {
    let count_bytes = bytes.get(80..84)?;
    let mut buf = [0u8; 4];
    if count_bytes.len() != buf.len() {
        return None;
    }
    buf.copy_from_slice(count_bytes);
    let count = u32::from_le_bytes(buf);
    let total = 84u64.checked_add(u64::from(count).checked_mul(50)?)?;
    let len = u64::try_from(bytes.len()).ok()?;
    if len == total { Some(count) } else { None }
}

fn starts_with_solid(bytes: &[u8]) -> bool {
    let mut index = 0usize;
    while bytes.get(index).is_some_and(|byte| byte.is_ascii_whitespace()) {
        index = match index.checked_add(1) {
            Some(next) => next,
            None => return false,
        };
    }
    let end = match index.checked_add(5) {
        Some(end) => end,
        None => return false,
    };
    let Some(head) = bytes.get(index..end) else {
        return false;
    };
    if !head.eq_ignore_ascii_case(b"solid") {
        return false;
    }
    match bytes.get(end) {
        None => true,
        Some(byte) => byte.is_ascii_whitespace(),
    }
}

fn parse_binary(bytes: &[u8], count: u32) -> Result<Mesh> {
    let count = usize::try_from(count).map_err(|_| invalid("too many triangles"))?;
    check_triangle_count(count)?;
    let mut triangles = Vec::with_capacity(count);
    let mut offset = 84usize;
    for _ in 0..count {
        skip(&mut offset, 12)?;
        let a = read_vec3(bytes, &mut offset)?;
        let b = read_vec3(bytes, &mut offset)?;
        let c = read_vec3(bytes, &mut offset)?;
        skip(&mut offset, 2)?;
        triangles.push(Triangle { a, b, c });
    }
    Mesh::new(triangles)
}

fn skip(offset: &mut usize, n: usize) -> Result<()> {
    *offset = offset.checked_add(n).ok_or_else(|| parse_error("binary STL offset overflow"))?;
    Ok(())
}

fn read_f32(bytes: &[u8], offset: usize) -> Result<f32> {
    let end = offset.checked_add(4).ok_or_else(|| parse_error("binary STL offset overflow"))?;
    let slice = bytes.get(offset..end).ok_or_else(|| parse_error("binary STL is truncated"))?;
    let mut buf = [0u8; 4];
    if slice.len() != buf.len() {
        return Err(parse_error("binary STL is truncated"));
    }
    buf.copy_from_slice(slice);
    Ok(f32::from_le_bytes(buf))
}

fn read_vec3(bytes: &[u8], offset: &mut usize) -> Result<Vec3> {
    let x = f64::from(read_f32(bytes, *offset)?);
    skip(offset, 4)?;
    let y = f64::from(read_f32(bytes, *offset)?);
    skip(offset, 4)?;
    let z = f64::from(read_f32(bytes, *offset)?);
    skip(offset, 4)?;
    Ok(Vec3::new(x, y, z))
}

struct Token {
    line: usize,
    start: usize,
    end: usize,
}

struct Cursor<'a> {
    bytes: &'a [u8],
    pos: usize,
    line: usize,
}

impl<'a> Cursor<'a> {
    fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, pos: 0, line: 1 }
    }

    fn skip_ws(&mut self) {
        while let Some(&byte) = self.bytes.get(self.pos) {
            if byte == b'\n' {
                self.line = self.line.saturating_add(1);
            } else if !byte.is_ascii_whitespace() {
                break;
            }
            match self.pos.checked_add(1) {
                Some(next) => self.pos = next,
                None => break,
            }
        }
    }

    fn peek_token(&mut self) -> Option<Token> {
        self.skip_ws();
        self.token_at(self.pos)
    }

    fn next_token(&mut self) -> Option<Token> {
        let token = self.peek_token()?;
        self.pos = token.end;
        Some(token)
    }

    fn token_at(&self, start: usize) -> Option<Token> {
        self.bytes.get(start)?;
        let mut end = start;
        while self.bytes.get(end).is_some_and(|byte| !byte.is_ascii_whitespace()) {
            end = end.checked_add(1)?;
        }
        if end == start { None } else { Some(Token { line: self.line, start, end }) }
    }

    fn text(&self, token: &Token) -> &'a [u8] {
        self.bytes.get(token.start..token.end).unwrap_or(b"")
    }
}

fn parse_ascii(bytes: &[u8]) -> Result<Mesh> {
    let mut cursor = Cursor::new(bytes);
    let mut triangles = Vec::new();
    loop {
        if cursor.peek_token().is_none() {
            break;
        }
        expect_kw(&mut cursor, b"solid")?;
        skip_until_keyword(&mut cursor, &[b"facet", b"endsolid"]);
        loop {
            if cursor.peek_token().is_none() {
                return Err(parse_at(cursor.line, "unexpected end of input inside solid"));
            }
            if is_kw(&mut cursor, b"endsolid") {
                expect_kw(&mut cursor, b"endsolid")?;
                skip_until_keyword(&mut cursor, &[b"solid"]);
                break;
            }
            let triangle = parse_facet(&mut cursor)?;
            push_triangle(&mut triangles, triangle)?;
        }
    }
    Mesh::new(triangles)
}

fn push_triangle(triangles: &mut Vec<Triangle>, triangle: Triangle) -> Result<()> {
    check_triangle_count(triangles.len().saturating_add(1))?;
    triangles.push(triangle);
    Ok(())
}

fn parse_facet(cursor: &mut Cursor<'_>) -> Result<Triangle> {
    expect_kw(cursor, b"facet")?;
    expect_kw(cursor, b"normal")?;
    for _ in 0..3 {
        let _ = expect_f64(cursor)?;
    }
    expect_kw(cursor, b"outer")?;
    expect_kw(cursor, b"loop")?;
    let a = expect_vertex(cursor)?;
    let b = expect_vertex(cursor)?;
    let c = expect_vertex(cursor)?;
    expect_kw(cursor, b"endloop")?;
    expect_kw(cursor, b"endfacet")?;
    Ok(Triangle { a, b, c })
}

fn expect_vertex(cursor: &mut Cursor<'_>) -> Result<Vec3> {
    expect_kw(cursor, b"vertex")?;
    Ok(Vec3::new(expect_f64(cursor)?, expect_f64(cursor)?, expect_f64(cursor)?))
}

fn expect_kw(cursor: &mut Cursor<'_>, keyword: &[u8]) -> Result<()> {
    let expected = keyword_str(keyword);
    let Some(token) = cursor.next_token() else {
        return Err(parse_at(cursor.line, format!("expected '{expected}', found end of input")));
    };
    let text = cursor.text(&token);
    if text.eq_ignore_ascii_case(keyword) {
        return Ok(());
    }
    Err(parse_at(token.line, format!("expected '{expected}', found '{}'", token_preview(text))))
}

fn expect_f64(cursor: &mut Cursor<'_>) -> Result<f64> {
    let Some(token) = cursor.next_token() else {
        return Err(parse_at(cursor.line, "expected a number, found end of input"));
    };
    let text = cursor.text(&token);
    let preview = token_preview(text);
    let utf8 = match std::str::from_utf8(text) {
        Ok(text) => text,
        Err(_) => return Err(parse_at(token.line, format!("expected a number, found '{preview}'"))),
    };
    let value: f64 = match utf8.parse() {
        Ok(value) => value,
        Err(_) => return Err(parse_at(token.line, format!("expected a number, found '{preview}'"))),
    };
    if value.is_finite() { Ok(value) } else { Err(parse_at(token.line, "coordinate is not finite")) }
}

fn is_kw(cursor: &mut Cursor<'_>, keyword: &[u8]) -> bool {
    match cursor.peek_token() {
        Some(token) => cursor.text(&token).eq_ignore_ascii_case(keyword),
        None => false,
    }
}

fn skip_until_keyword(cursor: &mut Cursor<'_>, keywords: &[&[u8]]) {
    loop {
        let Some(token) = cursor.peek_token() else {
            return;
        };
        let text = cursor.text(&token);
        if keywords.iter().any(|keyword| text.eq_ignore_ascii_case(keyword)) {
            return;
        }
        cursor.next_token();
    }
}

fn keyword_str(keyword: &[u8]) -> &str {
    match std::str::from_utf8(keyword) {
        Ok(text) => text,
        Err(_) => "keyword",
    }
}

fn token_preview(bytes: &[u8]) -> String {
    let end = bytes.len().min(64);
    let Some(slice) = bytes.get(..end) else {
        return String::new();
    };
    let mut text = String::from_utf8_lossy(slice).into_owned();
    if bytes.len() > end {
        text.push_str("...");
    }
    text
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Aabb, MeshError, Vec3};
    use proptest::prelude::*;

    fn xyz(x: f64, y: f64, z: f64) -> String {
        format!("{x} {y} {z}")
    }

    fn facet(a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> String {
        format!(
            "facet normal 0 0 0\nouter loop\nvertex {}\nvertex {}\nvertex {}\nendloop\nendfacet\n",
            xyz(a[0], a[1], a[2]),
            xyz(b[0], b[1], b[2]),
            xyz(c[0], c[1], c[2]),
        )
    }

    fn ascii_cube() -> String {
        let facets = [
            facet([0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [10.0, 10.0, 0.0]),
            facet([0.0, 0.0, 0.0], [10.0, 10.0, 0.0], [0.0, 10.0, 0.0]),
            facet([0.0, 0.0, 10.0], [10.0, 10.0, 10.0], [10.0, 0.0, 10.0]),
            facet([0.0, 0.0, 10.0], [0.0, 10.0, 10.0], [10.0, 10.0, 10.0]),
            facet([0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [10.0, 0.0, 10.0]),
            facet([0.0, 0.0, 0.0], [10.0, 0.0, 10.0], [0.0, 0.0, 10.0]),
            facet([0.0, 10.0, 0.0], [10.0, 10.0, 10.0], [10.0, 10.0, 0.0]),
            facet([0.0, 10.0, 0.0], [0.0, 10.0, 10.0], [10.0, 10.0, 10.0]),
            facet([0.0, 0.0, 0.0], [0.0, 0.0, 10.0], [0.0, 10.0, 10.0]),
            facet([0.0, 0.0, 0.0], [0.0, 10.0, 10.0], [0.0, 10.0, 0.0]),
            facet([10.0, 0.0, 0.0], [10.0, 10.0, 0.0], [10.0, 10.0, 10.0]),
            facet([10.0, 0.0, 0.0], [10.0, 10.0, 10.0], [10.0, 0.0, 10.0]),
        ];
        let mut text = String::from("solid cube\n");
        for facet in facets {
            text.push_str(&facet);
        }
        text.push_str("endsolid cube\n");
        text
    }

    fn binary_stl(tris: &[[[f32; 3]; 3]]) -> Vec<u8> {
        let count = u32::try_from(tris.len()).unwrap_or(0);
        let mut out = vec![0u8; 80];
        out.extend_from_slice(&count.to_le_bytes());
        for tri in tris {
            for _ in 0..3 {
                out.extend_from_slice(&0f32.to_le_bytes());
            }
            for vertex in tri {
                for component in vertex {
                    out.extend_from_slice(&component.to_le_bytes());
                }
            }
            out.extend_from_slice(&0u16.to_le_bytes());
        }
        out
    }

    fn assert_f32_vertex(original: Vec3, parsed: Vec3) {
        for (left, right) in [(original.x, parsed.x), (original.y, parsed.y), (original.z, parsed.z)] {
            let rounded = left as f32 as f64;
            assert_eq!(right.to_bits(), rounded.to_bits(), "{left} parsed as {right}, f32 {rounded}");
        }
    }

    /// Relative error of 1e-3, plus a small absolute slack so f32 underflow near zero still passes.
    fn within_rel(original: f64, parsed: f64) -> bool {
        let diff = (original - parsed).abs();
        let scale = original.abs().max(parsed.abs());
        diff <= 1e-3 * scale + 1e-6
    }

    fn within_rel_vec(original: Vec3, parsed: Vec3) -> bool {
        within_rel(original.x, parsed.x) && within_rel(original.y, parsed.y) && within_rel(original.z, parsed.z)
    }

    fn quantize_f32(vertex: Vec3) -> Vec3 {
        Vec3::new(vertex.x as f32 as f64, vertex.y as f32 as f64, vertex.z as f32 as f64)
    }

    fn quantize_triangle(triangle: Triangle) -> Triangle {
        Triangle { a: quantize_f32(triangle.a), b: quantize_f32(triangle.b), c: quantize_f32(triangle.c) }
    }

    #[test]
    fn ascii_cube_bounds_are_zero_to_ten() {
        let text = ascii_cube();
        let mesh = parse_stl(text.as_bytes()).expect("cube");
        assert_eq!(mesh.triangles().len(), 12);
        assert_eq!(mesh.degenerate_dropped(), 0);
        let bounds = mesh.bounds().expect("bounds");
        assert_eq!(bounds, Aabb::from_points([Vec3::new(0.0, 0.0, 0.0), Vec3::new(10.0, 10.0, 10.0)]).expect("box"));
        assert!(bounds.contains_xy(0.0, 0.0));
        assert!(bounds.contains_xy(10.0, 10.0));
        assert!(!bounds.contains_xy(-0.1, 5.0));
        for triangle in mesh.triangles() {
            for vertex in [triangle.a, triangle.b, triangle.c] {
                for component in [vertex.x, vertex.y, vertex.z] {
                    assert!(component == 0.0 || component == 10.0, "{component}");
                }
            }
        }
        let again = parse_stl(&write_stl_binary(&mesh)).expect("binary cube");
        assert_eq!(again.triangles().len(), 12);
        assert_eq!(again.bounds().expect("binary bounds"), bounds);
    }

    #[test]
    fn binary_roundtrip_matches_f32_precision() {
        let triangles = vec![
            Triangle { a: Vec3::new(0.1, -2.5, 3.0), b: Vec3::new(4.0, 5.5, -6.25), c: Vec3::new(-7.0, 8.0, 9.5) },
            Triangle { a: Vec3::new(1.0 / 3.0, 2.0 / 3.0, 1000.0), b: Vec3::new(-1000.0, 0.0, 0.5), c: Vec3::new(12.25, -0.125, 40.0) },
        ];
        let mesh = Mesh::new(triangles).expect("finite");
        assert_eq!(mesh.triangles().len(), 2);
        let mut bytes = write_stl_binary(&mesh);
        assert_eq!(bytes.len(), 84 + 50 * mesh.triangles().len());
        for index in 0..mesh.triangles().len() {
            let attr = 84 + index * 50 + 48;
            if let Some(slot) = bytes.get_mut(attr..attr + 2) {
                slot.copy_from_slice(&0xABCDu16.to_le_bytes());
            }
        }
        let parsed = parse_stl(&bytes).expect("roundtrip");
        assert_eq!(parsed.degenerate_dropped(), 0);
        assert_eq!(parsed.triangles().len(), mesh.triangles().len());
        for (original, parsed) in mesh.triangles().iter().zip(parsed.triangles().iter()) {
            assert_f32_vertex(original.a, parsed.a);
            assert_f32_vertex(original.b, parsed.b);
            assert_f32_vertex(original.c, parsed.c);
        }
    }

    #[test]
    fn binary_header_starting_with_solid_stays_binary() {
        let mesh = Mesh::new(vec![Triangle { a: Vec3::new(0.0, 0.0, 0.0), b: Vec3::new(1.0, 0.0, 0.0), c: Vec3::new(0.0, 1.0, 0.5) }]).expect("tri");
        let mut bytes = write_stl_binary(&mesh);
        let prefix = b"solid ";
        if let Some(slot) = bytes.get_mut(..prefix.len()) {
            slot.copy_from_slice(prefix);
        }
        let parsed = parse_stl(&bytes).expect("binary wins over the solid header");
        assert_eq!(parsed.triangles().len(), 1);
        assert_f32_vertex(mesh.triangles()[0].a, parsed.triangles()[0].a);
        let mut empty = vec![0u8; 84];
        if let Some(slot) = empty.get_mut(..prefix.len()) {
            slot.copy_from_slice(prefix);
        }
        let parsed_empty = parse_stl(&empty).expect("zero-count binary");
        assert!(parsed_empty.triangles().is_empty());
    }

    #[test]
    fn ascii_any_whitespace_case_and_multiple_solids() {
        let one_line = b"SOLID n FaCeT NoRmAl 0 0 1 OuTeR lOoP VeRtEx 0 0 0 vertex  1\t0  0 VERTEX 0 1 0 endLOOP endFACET endSoLiD n";
        let mesh = parse_stl(one_line).expect("one line");
        assert_eq!(mesh.triangles().len(), 1);
        assert_eq!(mesh.triangles()[0].b, Vec3::new(1.0, 0.0, 0.0));

        let crlf =
            "solid s\r\nfacet normal 0 0 1\r\nouter loop\r\nvertex 0 0 0\r\nvertex 2 0 0\r\nvertex 0 2 0\r\nendloop\r\nendfacet\r\nendsolid s\r\n";
        assert_eq!(parse_stl(crlf.as_bytes()).expect("crlf").triangles().len(), 1);

        let two = "\
solid first
facet normal 0 0 1
outer loop
vertex 0 0 0
vertex 1 0 0
vertex 0 1 0
endloop
endfacet
endsolid first name
solid
facet normal 0 0 1
outer loop
vertex 0 0 1
vertex 1 0 1
vertex 0 1 1
endloop
endfacet
endsolid
";
        let mesh = parse_stl(two.as_bytes()).expect("two solids");
        assert_eq!(mesh.triangles().len(), 2);
        let bounds = mesh.bounds().expect("bounds");
        assert_eq!(bounds.min(), Vec3::new(0.0, 0.0, 0.0));
        assert_eq!(bounds.max(), Vec3::new(1.0, 1.0, 1.0));

        let empty_solid = b"\n  solid blank\nendsolid blank\n";
        let empty = parse_stl(empty_solid).expect("empty solid");
        assert!(empty.triangles().is_empty());
        assert!(empty.bounds().is_err());
    }

    #[test]
    fn degenerate_triangles_in_stl_are_dropped() {
        let ascii = "\
solid s
facet normal 0 0 0
outer loop
vertex 0 0 0
vertex 0 0 0
vertex 0 0 0
endloop
endfacet
facet normal 0 0 1
outer loop
vertex 0 0 0
vertex 10 0 0
vertex 0 10 0
endloop
endfacet
endsolid s
";
        let mesh = parse_stl(ascii.as_bytes()).expect("ascii");
        assert_eq!(mesh.triangles().len(), 1);
        assert_eq!(mesh.degenerate_dropped(), 1);

        let bytes = binary_stl(&[[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]], [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]]);
        let mesh = parse_stl(&bytes).expect("binary");
        assert_eq!(mesh.triangles().len(), 1);
        assert_eq!(mesh.degenerate_dropped(), 1);
        assert_eq!(mesh.triangles()[0].c, Vec3::new(0.0, 1.0, 0.0));
    }

    #[test]
    fn ascii_errors_name_the_line() {
        let src = "\
solid bad
facet normal 0 0 0
outer loop
vertex 0 0 0
vertex 1 0 0
vertex 0 1
endloop
endfacet
endsolid bad
";
        let err = parse_stl(src.as_bytes()).expect_err("missing z");
        assert!(matches!(err, MeshError::Parse(_)), "{err}");
        assert!(err.to_string().contains("line 7"), "{err}");
        assert!(err.to_string().contains("endloop"), "{err}");

        let nan = "\
solid s
facet normal 0 0 1
outer loop
vertex nan 0 0
vertex 1 0 0
vertex 0 1 0
endloop
endfacet
endsolid s
";
        let err = parse_stl(nan.as_bytes()).expect_err("nan");
        assert!(err.to_string().contains("line 4"), "{err}");
        assert!(err.to_string().contains("not finite"), "{err}");

        let early = "solid bad\nfacet oops\n";
        let err = parse_stl(early.as_bytes()).expect_err("keyword");
        assert!(err.to_string().contains("line 2"), "{err}");
        assert!(err.to_string().contains("normal"), "{err}");
    }

    #[test]
    fn hostile_inputs_return_errors() {
        assert!(matches!(parse_stl(b""), Err(MeshError::Parse(_))));
        assert!(parse_stl(b"   \n\t").is_err());
        assert!(parse_stl(b"this is not an stl").is_err());
        assert!(parse_stl(b"solidity cube").is_err());
        assert!(parse_stl(b"solid").is_err());

        let valid = binary_stl(&[[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]]);
        let mut truncated = valid.clone();
        truncated.pop();
        assert!(parse_stl(&truncated).is_err());
        assert!(parse_stl(&valid[..20]).is_err());
        assert!(parse_stl(&[0u8; 83]).is_err());

        let mut wrong = valid.clone();
        wrong[80..84].copy_from_slice(&2u32.to_le_bytes());
        assert!(parse_stl(&wrong).is_err());
        let mut long = valid.clone();
        long.extend_from_slice(&[0u8; 50]);
        assert!(parse_stl(&long).is_err());

        let mut huge = vec![0u8; 84];
        huge[80..84].copy_from_slice(&u32::MAX.to_le_bytes());
        assert!(parse_stl(&huge).is_err());
        let mut huge_solid = vec![0u8; 128];
        huge_solid[..6].copy_from_slice(b"solid ");
        huge_solid[80..84].copy_from_slice(&u32::MAX.to_le_bytes());
        assert!(parse_stl(&huge_solid).is_err());

        let nan = f32::NAN;
        let inf = f32::INFINITY;
        assert!(parse_stl(&binary_stl(&[[[nan, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]])).is_err());
        assert!(parse_stl(&binary_stl(&[[[0.0, inf, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]])).is_err());
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(80))]

        #[test]
        fn random_triangle_soups_roundtrip_within_1e_3_relative(
            coords in proptest::collection::vec(
                (-1e3f64..1e3, -1e3f64..1e3, -1e3f64..1e3, -1e3f64..1e3, -1e3f64..1e3, -1e3f64..1e3, -1e3f64..1e3, -1e3f64..1e3, -1e3f64..1e3),
                1..200,
            )
        ) {
            let triangles = coords
                .into_iter()
                .map(|(ax, ay, az, bx, by, bz, cx, cy, cz)| Triangle {
                    a: Vec3::new(ax, ay, az),
                    b: Vec3::new(bx, by, bz),
                    c: Vec3::new(cx, cy, cz),
                })
                .collect();
            let mesh = Mesh::new(triangles).expect("coords are finite");
            let parsed = parse_stl(&write_stl_binary(&mesh)).expect("binary soup");
            // f32 rounding can push a triangle that was just above the area floor under it.
            let surviving: Vec<Triangle> = mesh.triangles().iter().copied().filter(|triangle| {
                Mesh::new(vec![quantize_triangle(*triangle)]).expect("quantized").triangles().len() == 1
            }).collect();
            prop_assert_eq!(parsed.triangles().len(), surviving.len());
            prop_assert_eq!(parsed.degenerate_dropped(), mesh.triangles().len() - surviving.len());
            for (original, parsed) in surviving.iter().zip(parsed.triangles().iter()) {
                prop_assert!(within_rel_vec(original.a, parsed.a));
                prop_assert!(within_rel_vec(original.b, parsed.b));
                prop_assert!(within_rel_vec(original.c, parsed.c));
            }
        }

        #[test]
        fn arbitrary_bytes_never_panic(bytes in proptest::collection::vec(any::<u8>(), 0..4096)) {
            let _ = parse_stl(&bytes);
        }
    }
}
