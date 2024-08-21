use std::{
    process::Command,
    mem::size_of,
    fs::{File, create_dir_all},
    io::prelude::*
};
use crate::types::*;
use crate::config::{H,W,AREA};


/* out */

unsafe fn any_as_u8_slice<T: Sized>(p: &T) -> &[u8] {
    ::core::slice::from_raw_parts(
        (p as *const T) as *const u8,
        ::core::mem::size_of::<T>(),
    )
}

#[repr(packed(1))]
struct BMPFileHeader {
    _ty: u16,
    _size: u32,
    _reserved1: u16,
    _reserved2: u16,
    _off_bits: u32
}

#[repr(packed(1))]
struct BMPInfoHeader {
    _size: u32,
    _width: i32,
    _height: i32,
    _planes: i16,
    _bit_count: i16,
    _compression: u32,
    _size_image: u32,
    _x_pels_per_meter: i32,
    _y_pels_per_meter: i32,
    _clr_used: u32,
    _clr_important: u32
}

const BMP_HEADER_SIZE: usize = size_of::<BMPFileHeader>() + size_of::<BMPInfoHeader>();

fn save_bitmap_image(path: &str, image: &Vec<u8>, width: usize, height: usize) {
    let mut file = File::create(path).expect("error opening file");

    let area = height*width;

    let file_header = BMPFileHeader {
        _ty: 0x4D42,
        _size: (BMP_HEADER_SIZE + area * size_of::<u8>() * 3) as u32,
        _reserved1: 0,
        _reserved2: 0,
        _off_bits: BMP_HEADER_SIZE as u32
    };

    let info_header = BMPInfoHeader {
        _size: size_of::<BMPInfoHeader>() as u32,
        _width: width as i32,
        _height: -(height as i32),
        _planes: 1,
        _bit_count: 24,
        _compression: 0,
        _size_image: (area * size_of::<u8>() * 3) as u32,
        _x_pels_per_meter: 0,
        _y_pels_per_meter: 0,
        _clr_used: 0,
        _clr_important: 0
    };

    file.write_all(unsafe { any_as_u8_slice(&file_header) }).expect("error writing u8 slice");
    file.write_all(unsafe { any_as_u8_slice(&info_header) }).expect("error writing u8 slice");
    dbg!(image.len() + BMP_HEADER_SIZE);
    file.write_all(image).expect("error writing u8 slice");
}

pub fn load_colors(path: &str) -> Colors {
    let mut f = File::open(path).expect("error opening file");
    let mut buf = vec![0; BMP_HEADER_SIZE + AREA*3];
    f.read_exact(&mut buf).expect("error reading file");
    let mut cs = vec![[0.0; 3]; AREA];
    for i in 0..AREA {
        cs[i][0] = (buf[BMP_HEADER_SIZE + 3*i  ] as f32) / 255.0;
        cs[i][1] = (buf[BMP_HEADER_SIZE + 3*i+1] as f32) / 255.0;
        cs[i][2] = (buf[BMP_HEADER_SIZE + 3*i+2] as f32) / 255.0;
    }
    cs
}

fn stitch_video_from_disk(
    inpath: &str,
    outpath: &str, 
    framerate: u32
) {
    Command::new("sh")
        .arg("-c")
        .arg(["ffmpeg",
            &format!("-framerate {}", framerate),
            "-pattern_type glob",
            &format!("-i '{}/*.bmp'", inpath),
            "-c:v libx264",
            "-pix_fmt yuv420p",
            outpath,
            "-y"].join(" "))
        .output()
        .expect(&format!("error stitching together video '{}.mp4'", outpath));
}

fn save_bitmap_video(
    path: &str, 
    video_buffer: &Vec<Vec<u8>>,
    width: usize,
    height: usize,
    framerate: u32
) {
    let area = video_buffer[0].len();
    if area != width * height * 3 {
        panic!("video_buffer[0].len() != width * height * 3 ({area} != {width} * {height} * 3)");
    }
    let tmppath = "/tmp/color_bitmap_buffer";
    create_dir_all(tmppath).expect("error creating buffer directory");
    for (i, buf) in video_buffer.iter().enumerate() {
        let frame_area = video_buffer[i].len();
        if frame_area != width * height * 3 {
            panic!("video_buffer[{i}].len() != width * height * 3 ({frame_area} != {width} * {height} * 3)");
        }
        assert_eq!(frame_area, area);
        save_bitmap_image(
            &format!("{}/{:04}.bmp", tmppath, i),
            buf,
            width,
            height
        );
    }
    stitch_video_from_disk(tmppath, path, framerate);
    Command::new("sh")
        .arg("-c")
        .arg(["rm -rf", &tmppath].join(" "))
        .output()
        .expect(&format!("error cleaning up buffer directory"));
}

fn depths_to_colors(src: &Depths) -> Colors {
    let mut out = vec![];
    for i in 0..src.len() {
        let c = [
            src[i], src[i], src[i]
        ];
        out.push(c);
    }
    out
}

fn colors_to_raw(c: &Colors) -> Vec<u8> {
    let mut out = vec![];
    for i in 0..c.len() {
        out.push((255.0 * c[i][0]) as u8);
        out.push((255.0 * c[i][1]) as u8);
        out.push((255.0 * c[i][2]) as u8);
    }
    out
}


// interface

pub fn save_colors(path: &str, c: &Colors) {
    let raw = colors_to_raw(c);
    save_bitmap_image(path, &raw, W, H);
}

pub fn save_colors2(path: &str, c1: &Colors, c2: &Colors) {
    let raw1 = colors_to_raw(c1);
    let raw2 = colors_to_raw(c2);
    let mut raw_combined = vec![];
    for y in 0..H {
        for x in 0..2*W {
            for i in 0..=2 {
                if x < W {
                    raw_combined.push(raw1[y*W*3 + x*3 + i]);
                } else {
                    raw_combined.push(raw2[y*W*3 + (x - W)*3 + i]);
                }
            }
        }
    }
    save_bitmap_image(path, &raw_combined, 2*W, H);
}

pub fn save_depths(path: &str, d: &Depths) {
    let c = depths_to_colors(d);
    save_colors(path, &c);
}

pub fn save_depths2(path: &str, d1: &Depths, d2: &Depths) {
    let c1 = depths_to_colors(d1);
    let c2 = depths_to_colors(d2);
    save_colors2(path, &c1, &c2);
}

pub fn save_colors_video(path: &str, cs: &Vec<Colors>, framerate: u32) {
    let raws = cs.iter().map(|c| colors_to_raw(c)).collect::<Vec<Vec<u8>>>();
    save_bitmap_video(path, &raws, W, H, framerate);
}

pub fn save_colors2_video(path: &str, cs1: &Vec<Colors>, cs2: &Vec<Colors>, framerate: u32) {
    assert_eq!(cs1.len(), cs2.len());
    let mut raws = vec![];
    for i in 0..cs1.len() {
        let raw1 = colors_to_raw(&cs1[i]);
        let raw2 = colors_to_raw(&cs2[i]);
        let mut raw_combined = vec![];
        for y in 0..H {
            for x in 0..2*W {
                for i in 0..=2 {
                    if x < W {
                        raw_combined.push(raw1[y*W*3 + x*3 + i]);
                    } else {
                        raw_combined.push(raw2[y*W*3 + (x - W)*3 + i]);
                    }
                }
            }
        }
        raws.push(raw_combined);
    }
    save_bitmap_video(path, &raws, 2*W, H, framerate);
}

pub fn save_depths_video(path: &str, ds: &Vec<Depths>, framerate: u32) {
    let raws = ds.iter().map(|d| colors_to_raw(&depths_to_colors(d))).collect::<Vec<Vec<u8>>>();
    save_bitmap_video(path, &raws, W, H, framerate);
}

pub fn save_depths2_video(path: &str, ds1: &Vec<Depths>, ds2: &Vec<Depths>, framerate: u32) {
    assert_eq!(ds1.len(), ds2.len());
    let mut raws = vec![];
    for i in 0..ds1.len() {
        let raw1 = colors_to_raw(&depths_to_colors(&ds1[i]));
        let raw2 = colors_to_raw(&depths_to_colors(&ds2[i]));
        let mut raw_combined = vec![];
        for y in 0..H {
            for x in 0..2*W {
                for i in 0..=2 {
                    if x < W {
                        raw_combined.push(raw1[y*W*3 + x*3 + i]);
                    } else {
                        raw_combined.push(raw2[y*W*3 + (x - W)*3 + i]);
                    }
                }
            }
        }
        raws.push(raw_combined);
    }
    save_bitmap_video(path, &raws, 2*W, H, framerate);
}