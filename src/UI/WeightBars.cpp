#include "WeightBars.h"

#include <algorithm>
#include <cfloat>
#include <cstdio>

#include "imgui.h"

void drawWeightTable(const char* id, int count, const char* (*name)(int, const void*), const float* values,
					 const void* context)
{
	// The name columns hug the widest label so the bars take the rest of the row.
	float nameWidth= 0.f;
	for (int index= 0; index < count; ++index)
		nameWidth= std::max(nameWidth, ImGui::CalcTextSize(name(index, context)).x);

	if (!ImGui::BeginTable(id, 4, ImGuiTableFlags_SizingStretchProp))
		return;
	ImGui::TableSetupColumn("nameA", ImGuiTableColumnFlags_WidthFixed, nameWidth);
	// Equal stretch weights keep both bar columns the same width.
	ImGui::TableSetupColumn("barA", ImGuiTableColumnFlags_WidthStretch, 1.f);
	ImGui::TableSetupColumn("nameB", ImGuiTableColumnFlags_WidthFixed, nameWidth);
	ImGui::TableSetupColumn("barB", ImGuiTableColumnFlags_WidthStretch, 1.f);
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
