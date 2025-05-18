use std::{
    io::{self, Read, Write},
    os::fd::{AsFd, AsRawFd, BorrowedFd},
};

use async_io::IoSafe;

/// A wrapper around [serialport::TTYPort] that implements the [AsFd] and [IoSafe] traits.
///
/// Useful for [smol::Async] async io adapter.
pub struct SerialPort(serialport::TTYPort);

impl SerialPort {
    #[expect(unused)]
    pub fn new(serial: serialport::TTYPort) -> Self {
        Self(serial)
    }
    pub fn new_async(serial: serialport::TTYPort) -> Result<smol::Async<Self>, io::Error> {
        smol::Async::new(Self(serial))
    }
}

impl AsFd for SerialPort {
    fn as_fd(&self) -> BorrowedFd<'_> {
        let fd = self.0.as_raw_fd();
        // SAFETY: fd lives when [SerialPort] lives, so it remain open for the duration of the returned BorrowedFd
        unsafe { BorrowedFd::borrow_raw(fd) }
    }
}

impl Read for SerialPort {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        self.0.read(buf)
    }
}

impl Write for SerialPort {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.0.write(buf)
    }

    fn flush(&mut self) -> io::Result<()> {
        self.0.flush()
    }
}

/// SAFETY: [SerialPort] only drop the underlying fd when it dropped.
unsafe impl IoSafe for SerialPort {}
