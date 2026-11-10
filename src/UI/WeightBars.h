#pragma once

// A table of names and progress bars, two per row, the readout shape the
// microphone's viseme block and the avatar mouth readout share. name
// gives the label of an index, with context passed through to it.
void drawWeightTable(const char* id, int count, const char* (*name)(int, const void*), const float* values,
					 const void* context);
