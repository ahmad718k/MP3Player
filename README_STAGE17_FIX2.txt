Stage17 FIX2 changes

1) Hybrid IMDCT overlap state is now persistent per channel across ALL granules and frames.
   Previous code allocated overlap as [granule][channel][32][18], which reset the
   overlap relationship at every granule and produced audible block discontinuities.
   It is now [channel][32][18].

2) Huffman region selection now compares MPEG spectral-line positions correctly.
   The previous decoder compared pair number directly against region boundaries,
   while MPEG region boundaries are spectral-line indices. The decoder now uses
   line = pair * 2 before selecting table 0/1/2.

3) Long-block region boundaries are passed as the actual MPEG scalefactor-band
   spectral-line boundaries, not divided by two. Short-block boundaries remain
   36 and 576 spectral lines.

These are targeted fixes. The decoder is still a development MPEG-1 Layer III
implementation and should be validated against the generated 5-second WAV.


STAGE17 FIX3 - SCFSI BIT-ALIGNMENT FIX
======================================

The MPEG-1 SCFSI bits are reused only for granule 1 NORMAL LONG blocks
(window_switching_flag == 0).  Switched blocks (block_type 1/2/3), including
start/end blocks, carry their own scalefactors and must not suppress the
scalefactor groups using SCFSI.

The previous implementation incorrectly applied SCFSI to block_type 1/3.
That could consume fewer scalefactor bits than are actually present and shift
the Huffman decoder cursor.  The resulting symptom was a large number of
apparently random Huffman errors such as table=28/25, table=27/24, etc.

FIX3 removes SCFSI reuse from all switched blocks and adds detailed Huffman
failure diagnostics (part2_3_length, scalefactor bits, big_values, block type,
region counts and bit cursor) so any remaining bitstream issue can be located
at the exact granule.
