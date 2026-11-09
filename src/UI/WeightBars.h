#pragma once

// A table of names and progress bars, two per row, the readout shape the
// Audio panel's viseme block and the OSC panel's mouth section share. name
// gives the label of an index, with context passed through to it.
void drawWeightTable(const char* id, int count, const char* (*name)(int, const void*), const float* values,
					 const void* context);
