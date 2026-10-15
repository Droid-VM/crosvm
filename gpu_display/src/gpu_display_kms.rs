// Copyright 2026 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! Direct scanout on Qualcomm SDE command-mode panels. `output_fence` with offset 0
//! signals PP_DONE (the current frame's DMA read is complete). Neither page-flip
//! events nor connector RETIRE_FENCE have that meaning. Video-mode panels are rejected.
use std::cell::RefCell;
use std::collections::BTreeMap;
use std::fs::{File, OpenOptions};
use std::os::fd::{AsRawFd, FromRawFd};
use std::path::Path;
use std::rc::{Rc, Weak};

use anyhow::{bail, ensure, Context, Result};
use base::{
    AsRawDescriptor, Event, FromRawDescriptor, RawDescriptor, SafeDescriptor, VolatileSlice,
};
use sync::Waitable;
use vm_control::gpu::DisplayParameters;

use crate::{
    DisplayExternalResourceImport, DisplayT, FlipToExtraInfo, GpuDisplayError,
    GpuDisplayFramebuffer, GpuDisplayResult, GpuDisplaySurface, SemaphoreTimepoint, SurfaceType,
    SysDisplayT, DRM_FORMAT_ABGR8888, DRM_FORMAT_ARGB8888, DRM_FORMAT_XBGR8888,
    DRM_FORMAT_XRGB8888,
};

const PLANE: u32 = 0xeeeeeeee;
const CRTC: u32 = 0xcccccccc;
const CONNECTOR: u32 = 0xc0c0c0c0;

// Fixed-width DRM UAPI structures (64-bit Linux/Android). ioctl payloads contain only integers.
macro_rules! uapi {
    ($name:ident { $($field:ident: $ty:ty),* $(,)? }) => {
        #[repr(C)] #[derive(Default)] struct $name { $($field: $ty),* }
    };
}
uapi!(Resources {
    fbs: u64,
    crtcs: u64,
    connectors: u64,
    encoders: u64,
    nfbs: u32,
    ncrtcs: u32,
    nconnectors: u32,
    nencoders: u32,
    minw: u32,
    maxw: u32,
    minh: u32,
    maxh: u32
});
uapi!(Mode {
    clock: u32,
    hdisplay: u16,
    hsync_start: u16,
    hsync_end: u16,
    htotal: u16,
    hskew: u16,
    vdisplay: u16,
    vsync_start: u16,
    vsync_end: u16,
    vtotal: u16,
    vscan: u16,
    refresh: u32,
    flags: u32,
    kind: u32,
    name: [u8; 32]
});
uapi!(Crtc {
    connectors: u64,
    nconnectors: u32,
    id: u32,
    fb: u32,
    x: u32,
    y: u32,
    gamma: u32,
    valid: u32,
    mode: Mode
});
uapi!(Connector {
    encoders: u64,
    modes: u64,
    props: u64,
    values: u64,
    nmodes: u32,
    nprops: u32,
    nencoders: u32,
    encoder: u32,
    id: u32,
    kind: u32,
    kind_id: u32,
    connection: u32,
    mmw: u32,
    mmh: u32,
    subpixel: u32,
    pad: u32
});
uapi!(PlaneResources {
    planes: u64,
    count: u32,
    pad: u32
});
uapi!(PlaneInfo {
    id: u32,
    crtc: u32,
    fb: u32,
    possible: u32,
    gamma: u32,
    nformats: u32,
    formats: u64
});
uapi!(Properties {
    props: u64,
    values: u64,
    count: u32,
    id: u32,
    kind: u32,
    pad: u32
});
uapi!(Property {
    values: u64,
    enums: u64,
    id: u32,
    flags: u32,
    name: [u8; 32],
    nvalues: u32,
    nenums: u32
});
uapi!(ClientCap {
    capability: u64,
    value: u64
});
uapi!(Prime {
    handle: u32,
    flags: u32,
    fd: i32
});
uapi!(GemClose {
    handle: u32,
    pad: u32
});
uapi!(Framebuffer {
    id: u32,
    width: u32,
    height: u32,
    fourcc: u32,
    flags: u32,
    handles: [u32; 4],
    pitches: [u32; 4],
    offsets: [u32; 4],
    modifiers: [u64; 4]
});
uapi!(Atomic {
    flags: u32,
    count: u32,
    objects: u64,
    counts: u64,
    props: u64,
    values: u64,
    reserved: u64,
    user_data: u64
});
uapi!(HeapAlloc {
    len: u64,
    fd: u32,
    fd_flags: u32,
    heap_flags: u64
});
uapi!(DmaSync { flags: u64 });

// Catch UAPI layout drift at compile time on the actual Android target.
const _: () = {
    assert!(std::mem::size_of::<Resources>() == 64);
    assert!(std::mem::size_of::<Mode>() == 68);
    assert!(std::mem::size_of::<Crtc>() == 104);
    assert!(std::mem::size_of::<Connector>() == 80);
    assert!(std::mem::size_of::<PlaneInfo>() == 32);
    assert!(std::mem::size_of::<PlaneResources>() == 16);
    assert!(std::mem::size_of::<Properties>() == 32);
    assert!(std::mem::size_of::<Property>() == 64);
    assert!(std::mem::size_of::<Framebuffer>() == 104);
    assert!(std::mem::size_of::<Atomic>() == 56);
    assert!(std::mem::size_of::<HeapAlloc>() == 24);
};

fn ioctl<T>(file: &File, number: u8, arg: &mut T) -> Result<()> {
    let direction = if matches!(number, 0x09 | 0x0d) {
        1u64
    } else {
        3u64
    };
    let request = (direction << 30)
        | ((std::mem::size_of::<T>() as u64) << 16)
        | ((b'd' as u64) << 8)
        | number as u64;
    // SAFETY: arg is a properly sized repr(C) UAPI payload, live through the ioctl.
    let rc = unsafe { libc::ioctl(file.as_raw_fd(), request as _, arg) };
    ensure!(
        rc >= 0,
        "DRM ioctl {number:#x}: {}",
        std::io::Error::last_os_error()
    );
    Ok(())
}
fn master(file: &File, acquire: bool) -> Result<()> {
    // SAFETY: SET/DROP_MASTER take no argument, file is an open DRM card.
    let rc = unsafe { libc::ioctl(file.as_raw_fd(), if acquire { 0x641e } else { 0x641f }) };
    ensure!(rc >= 0, "DRM master: {}", std::io::Error::last_os_error());
    Ok(())
}
fn text(bytes: &[u8]) -> String {
    String::from_utf8_lossy(bytes.split(|b| *b == 0).next().unwrap_or_default()).into_owned()
}
fn bounded(count: u32) -> Result<usize> {
    ensure!(count <= 4096, "unreasonable DRM object count {count}");
    Ok(count as usize)
}
fn properties(file: &File, id: u32, kind: u32) -> Result<BTreeMap<String, (u32, u64)>> {
    let mut p = Properties {
        id,
        kind,
        ..Default::default()
    };
    ioctl(file, 0xb9, &mut p)?;
    let mut ids = vec![0u32; bounded(p.count)?];
    let mut values = vec![0u64; ids.len()];
    p.props = ids.as_mut_ptr() as u64;
    p.values = values.as_mut_ptr() as u64;
    ioctl(file, 0xb9, &mut p)?;
    ensure!(
        p.count as usize == ids.len(),
        "DRM properties changed during enumeration"
    );
    let mut result = BTreeMap::new();
    for (id, value) in ids.into_iter().zip(values) {
        let mut prop = Property {
            id,
            ..Default::default()
        };
        ioctl(file, 0xaa, &mut prop)?;
        result.insert(text(&prop.name), (id, value));
    }
    Ok(result)
}
fn prop(props: &BTreeMap<String, (u32, u64)>, name: &str) -> Result<u32> {
    Ok(props
        .get(name)
        .with_context(|| format!("missing DRM property {name}"))?
        .0)
}

#[derive(Default)]
struct Request(BTreeMap<u32, Vec<(u32, u64)>>);
impl Request {
    fn add(&mut self, object: u32, property: u32, value: u64) {
        self.0.entry(object).or_default().push((property, value));
    }
    fn commit(&self, file: &File, flags: u32) -> Result<()> {
        let objects: Vec<u32> = self.0.keys().copied().collect();
        let counts: Vec<u32> = self.0.values().map(|p| p.len() as u32).collect();
        let props: Vec<u32> = self.0.values().flatten().map(|p| p.0).collect();
        let values: Vec<u64> = self.0.values().flatten().map(|p| p.1).collect();
        ioctl(
            file,
            0xbc,
            &mut Atomic {
                flags,
                count: objects.len() as u32,
                objects: objects.as_ptr() as u64,
                counts: counts.as_ptr() as u64,
                props: props.as_ptr() as u64,
                values: values.as_ptr() as u64,
                ..Default::default()
            },
        )
    }
}
struct Plane {
    id: u32,
    props: BTreeMap<String, (u32, u64)>,
    formats: Vec<u32>,
}
impl Plane {
    fn off(&self, req: &mut Request) {
        req.add(self.id, self.props["FB_ID"].0, 0);
        req.add(self.id, self.props["CRTC_ID"].0, 0);
    }
    fn set(&self, req: &mut Request, crtc: u32, fb: u32, src: [u32; 4], dst: [i32; 4]) {
        // SDE retains Android's private scaler/pixel-extension state after detach.
        // A null scaler_v2 pointer clears that state and lets the driver derive
        // scaling from this frame's source/destination rectangles.
        if let Some((scaler, _)) = self.props.get("scaler_v2") {
            req.add(self.id, *scaler, 0);
        }
        for (name, value) in [
            ("FB_ID", fb as u64),
            ("CRTC_ID", crtc as u64),
            ("SRC_X", (src[0] as u64) << 16),
            ("SRC_Y", (src[1] as u64) << 16),
            ("SRC_W", (src[2] as u64) << 16),
            ("SRC_H", (src[3] as u64) << 16),
            ("CRTC_X", dst[0] as i64 as u64),
            ("CRTC_Y", dst[1] as i64 as u64),
            ("CRTC_W", dst[2] as u64),
            ("CRTC_H", dst[3] as u64),
        ] {
            req.add(self.id, self.props[name].0, value);
        }
    }
}
struct Gem {
    file: Rc<File>,
    handle: u32,
}
impl Drop for Gem {
    fn drop(&mut self) {
        let _ = ioctl(
            &self.file,
            0x09,
            &mut GemClose {
                handle: self.handle,
                pad: 0,
            },
        );
    }
}
struct Buffer {
    gem: Rc<Gem>,
    fb: u32,
    width: u32,
    height: u32,
    fourcc: u32,
}
impl Drop for Buffer {
    fn drop(&mut self) {
        let _ = ioctl(&self.gem.file, 0xaf, &mut self.fb);
    }
}
struct CpuBuffer {
    file: File,
    bytes: Vec<u8>,
    buffer: Rc<Buffer>,
}
impl CpuBuffer {
    fn upload(&self) -> Result<()> {
        let fd = self.file.as_raw_fd();
        // SAFETY: mmap maps the complete allocation, which stays alive in self.file.
        let map = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                self.bytes.len(),
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                fd,
                0,
            )
        };
        ensure!(map != libc::MAP_FAILED, "map DMA buffer failed");
        let mut start = DmaSync { flags: 2 }; // START | WRITE
                                              // SAFETY: DMA_BUF_IOCTL_SYNC receives a valid u64 payload.
        let rc = unsafe { libc::ioctl(fd, 0x40086200u64 as _, &mut start) };
        if rc == 0 {
            // SAFETY: the mapping and source both cover bytes.len(), and do not overlap.
            unsafe {
                std::ptr::copy_nonoverlapping(self.bytes.as_ptr(), map.cast(), self.bytes.len());
            }
        }
        let mut end = DmaSync { flags: 2 | 4 }; // END | WRITE
                                                // SAFETY: same valid payload and mapping as above. Always unmap, including on error.
        let finish = unsafe { libc::ioctl(fd, 0x40086200u64 as _, &mut end) };
        unsafe {
            libc::munmap(map, self.bytes.len());
        }
        ensure!(
            rc == 0 && finish == 0,
            "DMA buffer CPU synchronization failed"
        );
        Ok(())
    }
}

struct Device {
    file: Rc<File>,
    crtc: u32,
    width: u32,
    height: u32,
    planes: Vec<Plane>,
    old_planes: Vec<Plane>,
    split: bool,
    output_fence: u32,
    output_offset: u32,
    imports: BTreeMap<u32, Rc<Buffer>>,
    gems: BTreeMap<u32, Weak<Gem>>,
    current: Option<Rc<Buffer>>,
    pending: Option<SafeDescriptor>,
    cursor: Option<Rc<Buffer>>,
    cursor_xy: (i32, i32),
    cursor_visible: bool,
    main_surface: bool,
    cursor_surface: bool,
    gpu_frames: u64,
    cpu_frames: u64,
}
impl Device {
    fn open(path: &Path, split: bool) -> Result<Self> {
        let file = Rc::new(OpenOptions::new().read(true).write(true).open(path)?);
        ioctl(
            &file,
            0x0d,
            &mut ClientCap {
                capability: 2,
                value: 1,
            },
        )?;
        ioctl(
            &file,
            0x0d,
            &mut ClientCap {
                capability: 3,
                value: 1,
            },
        )?;
        master(&file, true)?;
        let mut res = Resources::default();
        ioctl(&file, 0xa0, &mut res)?;
        let mut crtcs = vec![0u32; bounded(res.ncrtcs)?];
        let mut connectors = vec![0u32; bounded(res.nconnectors)?];
        res.crtcs = crtcs.as_mut_ptr() as u64;
        res.connectors = connectors.as_mut_ptr() as u64;
        // Other arrays have null pointers and zero capacity.
        res.nfbs = 0;
        res.nencoders = 0;
        ioctl(&file, 0xa0, &mut res)?;
        ensure!(
            res.ncrtcs as usize == crtcs.len() && res.nconnectors as usize == connectors.len(),
            "DRM topology changed during enumeration"
        );
        let mut active = Vec::new();
        for id in connectors {
            let mut conn = Connector {
                id,
                ..Default::default()
            };
            ioctl(&file, 0xa7, &mut conn)?;
            if conn.kind != 16 || conn.connection != 1 {
                continue;
            }
            let props = properties(&file, id, CONNECTOR)?;
            let crtc = props.get("CRTC_ID").map(|p| p.1 as u32).unwrap_or(0);
            if crtc != 0 {
                ensure!(
                    props.get("autorefresh").map(|p| p.1) == Some(0),
                    "KMS requires command mode with autorefresh disabled"
                );
                active.push(crtc);
            }
        }
        ensure!(active.len() == 1, "expected one active DSI connector");
        let crtc = active[0];
        let index = crtcs
            .iter()
            .position(|id| *id == crtc)
            .context("CRTC index")?;
        ensure!(index < 32, "invalid CRTC index");
        let mut info = Crtc {
            id: crtc,
            ..Default::default()
        };
        ioctl(&file, 0xa1, &mut info)?;
        ensure!(
            info.valid != 0 && text(&info.mode.name).ends_with("cmd"),
            "KMS supports only SDE command-mode panels"
        );
        let width = info.mode.hdisplay as u32;
        let height = info.mode.vdisplay as u32;
        ensure!(
            width > 0 && height > 0 && width % 2 == 0,
            "invalid panel dimensions"
        );
        let cp = properties(&file, crtc, CRTC)?;
        let output_fence = prop(&cp, "output_fence")?;
        let output_offset = prop(&cp, "output_fence_offset")?;
        let mut pr = PlaneResources::default();
        ioctl(&file, 0xb5, &mut pr)?;
        let mut ids = vec![0u32; bounded(pr.count)?];
        pr.planes = ids.as_mut_ptr() as u64;
        ioctl(&file, 0xb5, &mut pr)?;
        ensure!(pr.count as usize == ids.len(), "plane topology changed");
        let mut planes = Vec::new();
        let mut old_planes = Vec::new();
        for id in ids {
            let mut pi = PlaneInfo {
                id,
                ..Default::default()
            };
            ioctl(&file, 0xb6, &mut pi)?;
            if pi.possible & (1 << index) == 0 {
                continue;
            }
            let mut formats = vec![0u32; bounded(pi.nformats)?];
            pi.formats = formats.as_mut_ptr() as u64;
            ioctl(&file, 0xb6, &mut pi)?;
            ensure!(
                pi.nformats as usize == formats.len(),
                "plane formats changed"
            );
            let props = properties(&file, id, PLANE)?;
            for name in [
                "FB_ID", "CRTC_ID", "SRC_X", "SRC_Y", "SRC_W", "SRC_H", "CRTC_X", "CRTC_Y",
                "CRTC_W", "CRTC_H",
            ] {
                prop(&props, name)?;
            }
            let kind = props.get("type").map(|p| p.1).unwrap_or(2);
            let plane = Plane { id, props, formats };
            if kind != 2
                && planes.len() < if split { 3 } else { 2 }
                && plane.formats.contains(&DRM_FORMAT_ABGR8888)
            {
                planes.push(plane);
            } else if pi.crtc == crtc {
                old_planes.push(plane);
            }
        }
        ensure!(
            planes.len() >= if split { 3 } else { 2 },
            "not enough compatible scanout planes"
        );
        base::info!(
            "kms: {}x{}@{} cmd, CRTC {}, planes {:?}, PP_DONE release fence",
            width,
            height,
            info.mode.refresh,
            crtc,
            planes.iter().map(|p| p.id).collect::<Vec<_>>()
        );
        Ok(Self {
            file,
            crtc,
            width,
            height,
            planes,
            old_planes,
            split,
            output_fence,
            output_offset,
            imports: BTreeMap::new(),
            gems: BTreeMap::new(),
            current: None,
            pending: None,
            cursor: None,
            cursor_xy: (0, 0),
            cursor_visible: false,
            main_surface: false,
            cursor_surface: false,
            gpu_frames: 0,
            cpu_frames: 0,
        })
    }
    fn import(
        &mut self,
        fd: i32,
        width: u32,
        height: u32,
        stride: u32,
        offset: u32,
        fourcc: u32,
    ) -> Result<Rc<Buffer>> {
        ensure!(
            width > 0 && height > 0 && width <= 16384 && height <= 16384,
            "invalid buffer dimensions"
        );
        ensure!(stride as u64 >= width as u64 * 4, "short buffer stride");
        ensure!(
            [
                DRM_FORMAT_ABGR8888,
                DRM_FORMAT_ARGB8888,
                DRM_FORMAT_XBGR8888,
                DRM_FORMAT_XRGB8888
            ]
            .contains(&fourcc),
            "unsupported scanout format"
        );
        let mut p = Prime {
            fd,
            ..Default::default()
        };
        ioctl(&self.file, 0x2e, &mut p)?;
        // PRIME returns the same handle on repeated imports. GEM_CLOSE is not reference-counted.
        let gem = self
            .gems
            .get(&p.handle)
            .and_then(Weak::upgrade)
            .unwrap_or_else(|| {
                let gem = Rc::new(Gem {
                    file: self.file.clone(),
                    handle: p.handle,
                });
                self.gems.insert(p.handle, Rc::downgrade(&gem));
                gem
            });
        let mut fb = Framebuffer {
            width,
            height,
            fourcc,
            ..Default::default()
        };
        fb.handles[0] = p.handle;
        fb.pitches[0] = stride;
        fb.offsets[0] = offset;
        ioctl(&self.file, 0xb8, &mut fb)?;
        Ok(Rc::new(Buffer {
            gem,
            fb: fb.id,
            width,
            height,
            fourcc,
        }))
    }
    fn cpu_buffer(&mut self, width: u32, height: u32, fourcc: u32) -> Result<CpuBuffer> {
        let size = (width as usize)
            .checked_mul(height as usize)
            .and_then(|n| n.checked_mul(4))
            .context("CPU buffer overflow")?;
        ensure!(
            size <= 256 * 1024 * 1024 && size > 0,
            "CPU buffer too large"
        );
        let heap = File::open("/dev/dma_heap/system")?;
        let mut alloc = HeapAlloc {
            len: size as u64,
            fd_flags: (libc::O_RDWR | libc::O_CLOEXEC) as u32,
            ..Default::default()
        };
        // SAFETY: DMA_HEAP_IOCTL_ALLOC writes a fixed-size allocation payload and returns a new fd.
        ensure!(
            unsafe { libc::ioctl(heap.as_raw_fd(), 0xc0184800u64 as _, &mut alloc) } == 0,
            "DMA heap allocation failed"
        );
        let file = unsafe { File::from_raw_fd(alloc.fd as i32) };
        let buffer = self.import(file.as_raw_fd(), width, height, width * 4, 0, fourcc)?;
        Ok(CpuBuffer {
            file,
            bytes: vec![0; size],
            buffer,
        })
    }
    fn wait(&mut self) -> Result<()> {
        if let Some(fence) = &self.pending {
            let mut p = libc::pollfd {
                fd: fence.as_raw_descriptor(),
                events: libc::POLLIN,
                revents: 0,
            };
            // SAFETY: p is a valid pollfd. Keep the fence and buffers pinned on a timeout.
            let n = unsafe { libc::poll(&mut p, 1, 1000) };
            ensure!(
                n > 0
                    && p.revents & libc::POLLIN != 0
                    && p.revents & (libc::POLLERR | libc::POLLNVAL) == 0,
                "SDE release fence failed/timed out"
            );
        }
        self.pending = None;
        Ok(())
    }
    fn present(&mut self, buffer: Rc<Buffer>) -> Result<SafeDescriptor> {
        self.wait()?;
        let count = if self.split { 2 } else { 1 };
        ensure!(
            self.planes[..count]
                .iter()
                .all(|p| p.formats.contains(&buffer.fourcc)),
            "scanout planes do not support buffer format"
        );
        let mut req = Request::default();
        for p in &self.old_planes {
            p.off(&mut req);
        }
        ensure!(
            buffer.width % count as u32 == 0,
            "split scanout needs an even source width"
        );
        let w = self.width / count as u32;
        let source_w = buffer.width / count as u32;
        for (i, p) in self.planes[..count].iter().enumerate() {
            p.set(
                &mut req,
                self.crtc,
                buffer.fb,
                [i as u32 * source_w, 0, source_w, buffer.height],
                [i as i32 * w as i32, 0, w as i32, self.height as i32],
            );
        }
        for (index, p) in self.planes.iter().enumerate() {
            if let Some((zpos, _)) = p.props.get("zpos") {
                req.add(p.id, *zpos, index as u64);
            }
            // The desktop is opaque even when the guest's unused alpha bytes are zero.
            // Never inherit Android's per-layer coverage blending for the main scanout.
            if let Some((alpha, _)) = p.props.get("alpha") {
                req.add(p.id, *alpha, 65535);
            }
            if let Some((blend, _)) = p.props.get("blend_op") {
                req.add(p.id, *blend, 1); // SDE opaque
            }
        }
        let cursor_plane = &self.planes[count];
        cursor_plane.off(&mut req);
        if let Some(cursor) = self.cursor.as_ref().filter(|_| self.cursor_visible) {
            let (x, y) = self.cursor_xy;
            let left = x.max(0);
            let top = y.max(0);
            let right = (x as i64 + cursor.width as i64).min(self.width as i64);
            let bottom = (y as i64 + cursor.height as i64).min(self.height as i64);
            if right > left as i64 && bottom > top as i64 {
                // Replace the off properties: no duplicate property entries in an atomic object.
                req.0.remove(&cursor_plane.id);
                if let Some((zpos, _)) = cursor_plane.props.get("zpos") {
                    req.add(cursor_plane.id, *zpos, count as u64);
                }
                ensure!(
                    cursor_plane.formats.contains(&cursor.fourcc),
                    "cursor format unsupported"
                );
                cursor_plane.set(
                    &mut req,
                    self.crtc,
                    cursor.fb,
                    [
                        (left as i64 - x as i64) as u32,
                        (top as i64 - y as i64) as u32,
                        (right - left as i64) as u32,
                        (bottom - top as i64) as u32,
                    ],
                    [
                        left,
                        top,
                        (right - left as i64) as i32,
                        (bottom - top as i64) as i32,
                    ],
                );
            }
        }
        // SDE's private output fence is a pointer to u64, unlike standard OUT_FENCE_PTR (s32).
        let mut output = u64::MAX;
        req.add(self.crtc, self.output_offset, 0);
        req.add(
            self.crtc,
            self.output_fence,
            (&mut output as *mut u64) as u64,
        );
        let result = req.commit(&self.file, 0x200); // NONBLOCK; completion is PP_DONE, not flip event.
        let fence = if output <= i32::MAX as u64 {
            // SAFETY: successful property handling returned a newly owned sync_file fd.
            Some(unsafe { SafeDescriptor::from_raw_descriptor(output as i32) })
        } else {
            None
        };
        result?;
        self.current = Some(buffer);
        self.old_planes.clear();
        let fence = fence.context("SDE did not return an output fence")?;
        self.pending = Some(fence.try_clone()?);
        Ok(fence)
    }
}
impl Drop for Device {
    fn drop(&mut self) {
        base::info!(
            "kms: frames dmabuf={} cpu={}",
            self.gpu_frames,
            self.cpu_frames
        );
        let _ = self.wait();
        let mut req = Request::default();
        for p in self.planes.iter().chain(self.old_planes.iter()) {
            p.off(&mut req);
        }
        if let Err(e) = req.commit(&self.file, 0) {
            base::error!("kms: detach: {e:#}");
        }
        self.current = None;
        self.cursor = None;
        self.imports.clear();
        let _ = master(&self.file, false);
    }
}

struct Surface {
    device: Rc<RefCell<Device>>,
    cursor: bool,
    width: u32,
    height: u32,
    fourcc: u32,
    cpu: Option<CpuBuffer>,
    completion: Option<SafeDescriptor>,
}
impl GpuDisplaySurface for Surface {
    fn framebuffer(&mut self) -> Option<GpuDisplayFramebuffer> {
        if self.cpu.is_none() {
            self.cpu =
                match self
                    .device
                    .borrow_mut()
                    .cpu_buffer(self.width, self.height, self.fourcc)
                {
                    Ok(b) => Some(b),
                    Err(e) => {
                        base::error!("kms: CPU buffer: {e:#}");
                        return None;
                    }
                };
        }
        let cpu = self.cpu.as_mut()?;
        Some(GpuDisplayFramebuffer::new(
            VolatileSlice::new(cpu.bytes.as_mut_slice()),
            self.width * 4,
            4,
            self.fourcc,
        ))
    }
    fn set_buffer_fourcc(&mut self, fourcc: u32) {
        if self.fourcc != fourcc {
            self.cpu = None;
            self.fourcc = fourcc;
        }
    }
    fn flip(&mut self) {
        let result = (|| -> Result<()> {
            let cpu = self.cpu.as_ref().context("missing CPU framebuffer")?;
            let mut dev = self.device.borrow_mut();
            dev.wait()?;
            cpu.upload()?;
            if self.cursor {
                dev.cursor = Some(cpu.buffer.clone());
                dev.cursor_visible = true;
            } else {
                let _ = dev.present(cpu.buffer.clone())?;
                dev.cpu_frames += 1;
                if dev.cpu_frames == 1 {
                    base::info!("kms: first CPU scanout frame");
                }
            }
            Ok(())
        })();
        if let Err(e) = result {
            base::error!("kms: CPU flip: {e:#}");
        }
    }
    fn flip_to(
        &mut self,
        id: u32,
        acquire: Option<SemaphoreTimepoint>,
        release: Option<SemaphoreTimepoint>,
        _info: Option<FlipToExtraInfo>,
    ) -> Result<Waitable> {
        ensure!(
            !self.cursor && acquire.is_none() && release.is_none(),
            "unsupported KMS flip"
        );
        let mut dev = self.device.borrow_mut();
        let buffer = dev.imports.get(&id).context("unknown KMS import")?.clone();
        self.completion = Some(dev.present(buffer)?);
        dev.gpu_frames += 1;
        if dev.gpu_frames == 1 {
            base::info!("kms: first direct dmabuf scanout; PP_DONE fence returned");
        }
        Ok(Waitable::signaled())
    }
    fn take_flip_completion_fence(&mut self) -> Option<SafeDescriptor> {
        self.completion.take()
    }
    // A cursor-only commit would re-read the main dmabuf after releasing it to the guest.
    // Stage cursor changes until the next main frame, when its producer fence has been waited.
    fn set_position(&mut self, x: i32, y: i32) {
        if self.cursor {
            self.device.borrow_mut().cursor_xy = (x, y);
        }
    }
    fn set_cursor_visible(&mut self, visible: bool) {
        if self.cursor {
            self.device.borrow_mut().cursor_visible = visible;
        }
    }
}
impl Drop for Surface {
    fn drop(&mut self) {
        let mut dev = self.device.borrow_mut();
        if self.cursor {
            dev.cursor_visible = false;
            dev.cursor_surface = false;
        } else {
            dev.main_surface = false;
        }
    }
}
pub struct DisplayKms {
    device: Rc<RefCell<Device>>,
    event: Event,
}
impl DisplayKms {
    pub fn new(path: &Path, split: bool) -> GpuDisplayResult<Self> {
        Ok(Self {
            device: Rc::new(RefCell::new(Device::open(path, split).map_err(|e| {
                GpuDisplayError::Listen(format!("KMS {}: {e:#}", path.display()))
            })?)),
            event: Event::new()?,
        })
    }
}
impl AsRawDescriptor for DisplayKms {
    fn as_raw_descriptor(&self) -> RawDescriptor {
        self.event.as_raw_descriptor()
    }
}
impl SysDisplayT for DisplayKms {}
impl DisplayT for DisplayKms {
    fn create_surface(
        &mut self,
        parent: Option<u32>,
        _id: u32,
        _scanout: Option<u32>,
        params: &DisplayParameters,
        _kind: SurfaceType,
    ) -> GpuDisplayResult<Box<dyn GpuDisplaySurface>> {
        let cursor = parent.is_some();
        // virtio-gpu cursors have no DisplayParameters; the passed default is a desktop size.
        let (width, height) = if cursor { (64, 64) } else { params.get_virtual_display_size() };
        let mut dev = self.device.borrow_mut();
        if width == 0 || height == 0 || width > 16384 || height > 16384
            || (cursor && dev.cursor_surface)
            || (!cursor && dev.main_surface)
        {
            return Err(GpuDisplayError::CreateSurface);
        }
        if cursor {
            dev.cursor_surface = true;
        } else {
            dev.main_surface = true;
        }
        Ok(Box::new(Surface {
            device: self.device.clone(),
            cursor,
            width,
            height,
            fourcc: DRM_FORMAT_ARGB8888,
            cpu: None,
            completion: None,
        }))
    }
    fn import_resource(
        &mut self,
        id: u32,
        _surface: u32,
        resource: DisplayExternalResourceImport,
    ) -> Result<()> {
        let DisplayExternalResourceImport::Dmabuf {
            descriptor,
            offset,
            stride,
            modifiers,
            linear_layout_verified,
            width,
            height,
            fourcc,
        } = resource
        else {
            bail!("KMS requires dmabuf");
        };
        ensure!(
            modifiers == 0 || (modifiers == 0x00ff_ffff_ffff_ffff && linear_layout_verified),
            "KMS requires verified linear layout"
        );
        let mut dev = self.device.borrow_mut();
        let buffer = dev.import(
            descriptor.as_raw_descriptor(),
            width,
            height,
            stride,
            offset,
            fourcc,
        )?;
        dev.imports.insert(id, buffer);
        Ok(())
    }
    fn release_import(&mut self, id: u32, _surface: u32) {
        self.device.borrow_mut().imports.remove(&id);
    }
}
