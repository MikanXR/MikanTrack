#include "WeightBars.h"

#include <algorithm>
#include <cfloat>
#include <cstdio>

#include "imgui.h"

void drawWeightTable(const char* id, int count, const char* (*name)(int, const void*), const float* values,
					 const void* context)
{
	if (!ImGui::BeginTable(id, 4, ImGuiTableFlags_SizingStretchProp))
		return;
	ImGui::TableSetupColumn("nameA", ImGuiTableColumnFlags_WidthFixed, 150.f);
	ImGui::TableSetupColumn("barA", ImGuiTableColumnFlags_WidthStretch);
	ImGui::TableSetupColumn("nameB", ImGuiTableColumnFlags_WidthFixed, 150.f);
	ImGui::TableSetupColumn("barB", ImGuiTableColumnFlags_WidthStretch);
	const int rows= (count + 1) / 2;
	char overlay[16];
	for (int row= 0; row < rows; ++row)
	{
		ImGui::TableNextRow();
		for (int half= 0; half < 2; ++half)
		{
			const int index= row + half * rows;
			ImGui::TableSetColumnIndex(half * 2);
			if (index >= count)
				continue;
			ImGui::TextDisabled("%s", name(index, context));
			ImGui::TableSetColumnIndex(half * 2 + 1);
			const float weight= std::clamp(values[index], 0.f, 1.f);
			snprintf(overlay, sizeof(overlay), "%.2f", weight);
			ImGui::ProgressBar(weight, ImVec2(-FLT_MIN, 0.f), overlay);
		}
	}
	ImGui::EndTable();
}
