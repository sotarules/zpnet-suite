ZPNet Suite project instructions
Place all new unit tests under the repository-root tests/ directory
(zpnet-suite/tests/), including tests for Teensy firmware.

Never create or supply unit tests under pnc/firmware/teensy/tests/.
This rule applies to direct edits, generated files, and unified patches.

Deliver patches as unified diffs that apply from the zpnet-suite repository root. The preferred format is a single copyable diff code block in the response so the user can use CLion’s Apply Patch from Clipboard. Include the complete patch, with all file headers and hunks intact. Keep explanations outside the block. Provide a downloadable .patch file only when requested.