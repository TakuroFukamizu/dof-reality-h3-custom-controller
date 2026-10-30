//! Standard COBS (Cheshire & Baker) with 0x00 frame delimiter.

/// COBS-encode `data` (no trailing delimiter).
///
/// A full 254-byte non-zero group ends with code 0xFF and no extra trailing
/// `0x01` block unless more input follows (matches the Wikipedia vectors).
/// Output length is at most `data.len() + data.len() / 254 + 1`.
pub fn encode(data: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(data.len() + data.len() / 254 + 2);
    let mut code_idx = 0;
    out.push(0); // placeholder for the first code byte
    let mut code: u8 = 1;
    for &b in data {
        if code == 0xFF {
            // Previous group is full; open a new one only now that more input exists.
            out[code_idx] = code;
            code_idx = out.len();
            out.push(0);
            code = 1;
        }
        if b == 0 {
            out[code_idx] = code;
            code_idx = out.len();
            out.push(0);
            code = 1;
        } else {
            out.push(b);
            code += 1;
        }
    }
    out[code_idx] = code;
    out
}

/// COBS-decode one encoded frame (without its 0x00 delimiter).
///
/// Returns `None` for empty input, an embedded 0x00, or a code byte that runs
/// past the end. A trailing `0x01` after a final 0xFF group is accepted.
pub fn decode(encoded: &[u8]) -> Option<Vec<u8>> {
    if encoded.is_empty() {
        return None;
    }
    let mut out = Vec::with_capacity(encoded.len());
    let mut i = 0;
    while i < encoded.len() {
        let code = *encoded.get(i)?;
        if code == 0 {
            return None;
        }
        i += 1;
        let end = i + code as usize - 1;
        let block = encoded.get(i..end)?;
        if block.contains(&0) {
            return None;
        }
        out.extend_from_slice(block);
        i = end;
        // A group shorter than 254 implies a zero, except after the final group.
        if code < 0xFF && i < encoded.len() {
            out.push(0);
        }
    }
    Some(out)
}

/// Streaming splitter: feed raw USB bytes, get complete encoded frames
/// (delimiter stripped). Frames longer than `max_encoded_len` are dropped and the
/// accumulator skips to the next 0x00 (resync); `dropped()` counts them.
/// Empty frames (consecutive 0x00) are ignored silently.
#[derive(Debug)]
pub struct FrameAccumulator {
    buf: Vec<u8>,
    max_encoded_len: usize,
    /// True while discarding an oversize frame until the next 0x00.
    skipping: bool,
    dropped: u64,
}

impl FrameAccumulator {
    pub fn new(max_encoded_len: usize) -> Self {
        Self {
            buf: Vec::with_capacity(max_encoded_len),
            max_encoded_len,
            skipping: false,
            dropped: 0,
        }
    }

    /// Push bytes; returns every complete encoded frame found (may be empty).
    pub fn push(&mut self, bytes: &[u8]) -> Vec<Vec<u8>> {
        let mut frames = Vec::new();
        for &b in bytes {
            if b == 0 {
                if self.skipping {
                    self.skipping = false;
                } else if !self.buf.is_empty() {
                    frames.push(std::mem::take(&mut self.buf));
                    self.buf.reserve(self.max_encoded_len);
                }
            } else if self.skipping {
                // discard until delimiter
            } else if self.buf.len() >= self.max_encoded_len {
                self.skipping = true;
                self.dropped += 1;
                self.buf.clear();
            } else {
                self.buf.push(b);
            }
        }
        frames
    }

    /// Number of frames dropped for being oversize.
    pub fn dropped(&self) -> u64 {
        self.dropped
    }

    /// Discard any partial frame (e.g. after the port is reopened).
    /// The next bytes are treated as the start of a new frame.
    pub fn clear(&mut self) {
        self.buf.clear();
        self.skipping = false;
    }
}
