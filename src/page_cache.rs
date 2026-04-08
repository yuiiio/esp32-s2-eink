//! Page cache for e-ink reader
//! Manages up to 5 page buffers in PSRAM for prefetch and backward cache

extern crate alloc;
use alloc::boxed::Box;

use crate::eink::TWO_BPP_BUF_SIZE;

/// Maximum number of cached pages (limited by 2MB PSRAM)
/// 4 buffers × 379KB = 1.5MB, + prev_display_buf 379KB = 1.9MB
pub const CACHE_SIZE: usize = 4;

/// Cached page identifier
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct PageId {
    pub dir: u16,
    pub page: u16,
}

impl PageId {
    pub fn new(dir: u16, page: u16) -> Self {
        Self { dir, page }
    }

    /// Get next page ID within same directory
    pub fn next(&self, max_pages: u16, max_dirs: u16) -> Self {
        if self.page + 1 < max_pages {
            Self::new(self.dir, self.page + 1)
        } else if self.dir < max_dirs {
            Self::new(self.dir + 1, 0)
        } else {
            Self::new(1, 0) // wrap around
        }
    }

    /// Get previous page ID within same directory
    pub fn prev(&self, max_pages_fn: impl Fn(u16) -> u16, max_dirs: u16) -> Self {
        if self.page > 0 {
            Self::new(self.dir, self.page - 1)
        } else if self.dir > 1 {
            let prev_dir = self.dir - 1;
            let prev_max = max_pages_fn(prev_dir);
            Self::new(prev_dir, prev_max.saturating_sub(1))
        } else {
            let last_dir = max_dirs;
            let last_max = max_pages_fn(last_dir);
            Self::new(last_dir, last_max.saturating_sub(1)) // wrap around
        }
    }
}

/// Cache entry
struct CacheEntry {
    buffer: Box<[u8; TWO_BPP_BUF_SIZE]>,
    page_id: Option<PageId>,
}

impl CacheEntry {
    fn new() -> Self {
        Self {
            buffer: Box::new([0u8; TWO_BPP_BUF_SIZE]),
            page_id: None,
        }
    }
}

/// Page cache managing multiple buffers
pub struct PageCache {
    entries: [CacheEntry; CACHE_SIZE],
    /// Index of the "current" page in the cache (used for display)
    current_idx: usize,
    /// Index of the "previous" displayed page (for reverse waveform)
    prev_idx: usize,
}

impl PageCache {
    pub fn new() -> Self {
        Self {
            entries: core::array::from_fn(|_| CacheEntry::new()),
            current_idx: 0,
            prev_idx: 0,
        }
    }

    /// Get buffer for a specific page if cached
    pub fn get(&self, page_id: PageId) -> Option<&[u8; TWO_BPP_BUF_SIZE]> {
        self.entries
            .iter()
            .find(|e| e.page_id == Some(page_id))
            .map(|e| e.buffer.as_ref())
    }

    /// Check if page is cached
    pub fn contains(&self, page_id: PageId) -> bool {
        self.entries.iter().any(|e| e.page_id == Some(page_id))
    }

    /// Get the current display buffer
    pub fn current_buffer(&self) -> &[u8; TWO_BPP_BUF_SIZE] {
        &self.entries[self.current_idx].buffer
    }

    /// Get the previous buffer (for reverse display)
    pub fn prev_buffer(&self) -> &[u8; TWO_BPP_BUF_SIZE] {
        &self.entries[self.prev_idx].buffer
    }

    /// Mark current page as displayed (moves current to prev)
    /// Call this after displaying the current page
    pub fn mark_displayed(&mut self) {
        self.prev_idx = self.current_idx;
    }

    /// Find or allocate a buffer for loading a page
    /// Returns (buffer_mut, was_cached)
    pub fn get_or_alloc(&mut self, page_id: PageId) -> (&mut [u8; TWO_BPP_BUF_SIZE], bool) {
        // Check if already cached
        if let Some(idx) = self.entries.iter().position(|e| e.page_id == Some(page_id)) {
            self.current_idx = idx;
            return (&mut self.entries[idx].buffer, true);
        }

        // Find an empty slot or LRU slot (furthest from current)
        let alloc_idx = self.find_alloc_slot(page_id);
        self.entries[alloc_idx].page_id = Some(page_id);
        self.current_idx = alloc_idx;
        (&mut self.entries[alloc_idx].buffer, false)
    }

    /// Find best slot to allocate for a new page
    fn find_alloc_slot(&self, target: PageId) -> usize {
        // First, try to find empty slot (but not prev_idx which holds previous display)
        if let Some(idx) = self.entries.iter().position(|e| e.page_id.is_none()) {
            if idx != self.prev_idx {
                return idx;
            }
        }

        // Otherwise, evict the page furthest from target
        // Never evict prev_idx (needed for reverse waveform display)
        let mut best_idx = (self.prev_idx + 1) % CACHE_SIZE;
        let mut best_dist = 0i32;

        for (idx, entry) in self.entries.iter().enumerate() {
            // Don't evict prev buffer
            if idx == self.prev_idx {
                continue;
            }
            if let Some(cached_id) = entry.page_id {
                let dist = page_distance(cached_id, target);
                if dist > best_dist {
                    best_dist = dist;
                    best_idx = idx;
                }
            }
        }
        best_idx
    }

    /// Prefetch a page into cache without setting it as current
    /// Returns buffer to fill if not already cached
    pub fn prefetch_slot(&mut self, page_id: PageId) -> Option<&mut [u8; TWO_BPP_BUF_SIZE]> {
        // Already cached?
        if self.contains(page_id) {
            return None;
        }

        // Find slot for prefetch (don't evict current)
        let alloc_idx = self.find_prefetch_slot(page_id);
        self.entries[alloc_idx].page_id = Some(page_id);
        Some(&mut self.entries[alloc_idx].buffer)
    }

    /// Find slot for prefetch (avoid current buffer)
    fn find_prefetch_slot(&self, target: PageId) -> usize {
        // First, try to find empty slot
        if let Some(idx) = self.entries.iter().position(|e| e.page_id.is_none()) {
            return idx;
        }

        // Evict furthest page, but not current
        let mut best_idx = (self.current_idx + 1) % CACHE_SIZE;
        let mut best_dist = 0i32;

        for (idx, entry) in self.entries.iter().enumerate() {
            if idx == self.current_idx {
                continue; // Don't evict current
            }
            if let Some(cached_id) = entry.page_id {
                let dist = page_distance(cached_id, target);
                if dist > best_dist {
                    best_dist = dist;
                    best_idx = idx;
                }
            }
        }
        best_idx
    }

    /// Mark a prefetched page as current (when navigating to it)
    pub fn set_current(&mut self, page_id: PageId) -> bool {
        if let Some(idx) = self.entries.iter().position(|e| e.page_id == Some(page_id)) {
            self.current_idx = idx;
            true
        } else {
            false
        }
    }

    /// Invalidate all cache entries for a directory
    pub fn invalidate_dir(&mut self, dir: u16) {
        for entry in &mut self.entries {
            if let Some(id) = entry.page_id {
                if id.dir == dir {
                    entry.page_id = None;
                }
            }
        }
    }
}

/// Calculate "distance" between two pages for eviction policy
fn page_distance(a: PageId, b: PageId) -> i32 {
    if a.dir == b.dir {
        (a.page as i32 - b.page as i32).abs()
    } else {
        // Different directory = large distance
        1000 + (a.dir as i32 - b.dir as i32).abs() * 100
    }
}
