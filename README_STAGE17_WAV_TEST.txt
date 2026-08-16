STAGE17 WAV TEST FIX

This revision fixes the zero-byte WAV failure path.

Changes:
- Removed unavailable esp_vfs_fat_info() call.
- The raw FAT 4 KB erase buffer is static instead of being allocated on the VFS task stack.
- WAV capture prints diagnostics before fopen(), after fopen(), and after committing the 44-byte placeholder.
- test.wav is truncated/replaced on every run.
- WAV placeholder is flushed immediately.
- PCM data is flushed while capture is running.
- Final WAV header is rewritten with the actual PCM byte count.

Expected log:
WAV: opening file...
WAV: file opened
WAV: 44-byte placeholder committed
WAV: first PCM frame: rate=44100 channels=2 samples=1152
...
WAV samples : 220500
WAV data    : 882000 bytes
WAV duration: 5.000 sec
