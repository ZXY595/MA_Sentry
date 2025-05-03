use std::{
    io::{Read, Write},
    os::fd::{AsFd, AsRawFd, BorrowedFd},
    sync::Arc,
};

use async_io::IoSafe;

pub struct TTYRx {
    port: Arc<serialport::TTYPort>,
}

pub struct TTYTx {
    port: Arc<serialport::TTYPort>,
}

fn port_ref_to_raw_mut(port: &Arc<serialport::TTYPort>) -> *mut serialport::TTYPort {
    Arc::as_ptr(port) as *mut serialport::TTYPort
}

impl Read for TTYRx {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        let port = port_ref_to_raw_mut(&self.port);
        // SAFETY: serialport::TTYPort::read never mutates the port
        // and call read and write in the same time is safe
        unsafe { (*port).read(buf) }
    }
}

impl Write for TTYTx {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        let port = port_ref_to_raw_mut(&self.port);
        // SAFETY: serialport::TTYPort::write never mutates the port
        // and call read and write in the same time is safe
        unsafe { (*port).write(buf) }
    }

    fn flush(&mut self) -> std::io::Result<()> {
        let port = port_ref_to_raw_mut(&self.port);
        // SAFETY: serialport::TTYPort::flush never mutates the port
        unsafe { (*port).flush() }
    }
}

impl AsFd for TTYRx {
    fn as_fd(&self) -> BorrowedFd<'_> {
        let fd = self.port.as_raw_fd();
        unsafe { BorrowedFd::borrow_raw(fd) }
    }
}

impl AsFd for TTYTx {
    fn as_fd(&self) -> BorrowedFd<'_> {
        let fd = self.port.as_raw_fd();
        unsafe { BorrowedFd::borrow_raw(fd) }
    }
}

// SAFETY: serialport::TTYPort only drop the underlying fd when no more reference to it exists
unsafe impl IoSafe for TTYRx {}
// SAFETY: serialport::TTYPort only drop the underlying fd when no more reference to it exists
unsafe impl IoSafe for TTYTx {}

/// split a serial port into a rx and a tx
/// 
/// the inner ttyport is dropped when the rx and tx are dropped
pub fn tty_split(port: serialport::TTYPort) -> (TTYRx, TTYTx) {
    let port = Arc::new(port);
    (
        TTYRx {
            port: Arc::clone(&port),
        },
        TTYTx { port },
    )
}
