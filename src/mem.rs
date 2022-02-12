use core::alloc::Layout;

use alloc_cortex_m::CortexMHeap;

use crate::{_print, print, println};

#[global_allocator]
pub static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

#[alloc_error_handler]
fn out_of_memory(l: Layout) -> ! {
    println!("Out of memory: {:?}", &l);
    loop {}
}
