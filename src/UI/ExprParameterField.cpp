#include "ExprParameterField.h"

#include <algorithm>
#include <string>

#include "imgui.h"

#include "FaceTabWidgets.h"
#include "LocText.h"

namespace
{
// The one text being typed. ImGui has one active item at a time, so one
// buffer serves every field, keyed by the input's id; lastFrame lets a field
// that stopped being drawn while typed in (another chain selected) drop it.
struct EditBuffer
{
	ImGuiID id= 0;
	std::string text;
	int lastFrame= -1;
};
EditBuffer s_edit;

bool isEditing(ImGuiID textId)
{
	return s_edit.id == textId && s_edit.lastFrame >= ImGui::GetFrameCount() - 1;
}

void drawHelpMarker(const ExprInputTable& table)
{
	ImGui::TextDisabled("(?)");
	if (ImGui::BeginItemTooltip())
	{
		ImGui::TextUnformatted(locText("exprField.helpText"));
		for (int slot= 0; slot < table.getCount(); ++slot)
			ImGui::BulletText("%s", table.getName(slot).c_str());
		ImGui::EndTooltip();
	}
}

// The live value under the text: the bar within the range and the number
// beside it, the fallback marked
void drawLiveBar(const ExprParameter& parameter, float minValue, float maxValue, const char* valueFormat,
				 bool bCompileError, const ExprParameterLiveValue& live, float width)
{
	bool bHave= false;
	bool bFallback= false;
	float value= 0.f;
	if (bCompileError)
	{
		bHave= true;
		bFallback= true;
		value= std::max(minValue, std::min(parameter.value, maxValue));
	}
	else if (live.bValid)
	{
		bHave= true;
		bFallback= !live.bFromExpression;
		value= live.value;
	}
	const float range= maxValue - minValue;
	const float fraction= bHave && range > 0.f ? std::clamp((value - minValue) / range, 0.f, 1.f) : 0.f;
	const float height= std::max(4.f, ImGui::GetTextLineHeight() * 0.35f);
	ImGui::ProgressBar(fraction, ImVec2(width, height), "");
	ImGui::SameLine(0.f, ImGui::GetStyle().ItemInnerSpacing.x);
	if (!bHave)
	{
		ImGui::TextDisabled("%s", locText("exprField.noValueLabel"));
		ImGui::SetItemTooltip("%s", locText("exprField.noValueTooltip"));
		return;
	}
	ImGui::Text(valueFormat, value);
	if (bFallback)
	{
		ImGui::SameLine();
		ImGui::TextColored(FaceTabWidgets::k_warningColor, "%s", locText("exprField.fallbackLabel"));
		ImGui::SetItemTooltip("%s", locText("exprField.fallbackTooltip"));
	}
	else if (live.rawValue != live.value)
	{
		ImGui::SameLine();
		ImGui::TextDisabled("%s", locFormat("exprField.rawFmt", live.rawValue).c_str());
		ImGui::SetItemTooltip("%s", locText("exprField.rawTooltip"));
	}
}
} // namespace

bool drawExprParameterField(const char* id, const char* label, ExprParameter& ioParameter, float minValue,
							float maxValue, const char* valueFormat, const char* tooltip, const ExprInputTable& table,
							const ExprParameterLiveValue& live)
{
	ImGui::PushID(id);
	const ImGuiStyle& style= ImGui::GetStyle();
	const float spacing= style.ItemInnerSpacing.x;
	const float fxWidth= ImGui::CalcTextSize("fx").x + style.FramePadding.x * 2.f;
	const float fieldWidth= std::max(ImGui::CalcItemWidth() - fxWidth - spacing, 40.f);
	const ImGuiID textId= ImGui::GetID("##expression");
	bool bCommit= false;
	bool bCompileError= false;
	FaceExprCompileResult compiled;

	if (!ioParameter.bUseExpression)
	{
		float value= ioParameter.value;
		ImGui::SetNextItemWidth(fieldWidth);
		if (ImGui::SliderFloat("##value", &value, minValue, maxValue, valueFormat))
		{
			ioParameter.value= value;
			bCommit= true;
		}
		if (tooltip != nullptr)
			ImGui::SetItemTooltip("%s", tooltip);
	}
	else
	{
		// The text shown: the one being typed in this field, else the stored
		// one. ImGui keeps its own copy while the input is active and writes
		// it back on an edit, so the buffer follows it frame to frame.
		std::string text= isEditing(textId) ? s_edit.text : ioParameter.expression;
		compiled= compileExprParameter(text, table);
		const FaceExprCompileError* error= compiled.errors.empty() ? nullptr : &compiled.errors.front();
		bCompileError= error != nullptr;
		FaceTabWidgets::textInputWithErrorSpan("##expression", text, error, fieldWidth);
		if (ImGui::IsItemActive())
		{
			s_edit.id= textId;
			s_edit.text= text;
			s_edit.lastFrame= ImGui::GetFrameCount();
		}
		if (ImGui::IsItemDeactivated())
		{
			// Enter or leaving the input commits; Escape restores the stored
			// text, which commits nothing
			if (ImGui::IsItemDeactivatedAfterEdit() && text != ioParameter.expression)
			{
				ioParameter.expression= text;
				bCommit= true;
			}
			if (s_edit.id == textId)
				s_edit= EditBuffer();
		}
		if (tooltip != nullptr)
			ImGui::SetItemTooltip("%s", tooltip);
	}

	ImGui::SameLine(0.f, spacing);
	const bool bLit= ioParameter.bUseExpression;
	if (bLit)
		ImGui::PushStyleColor(ImGuiCol_Button, style.Colors[ImGuiCol_ButtonActive]);
	if (ImGui::Button("fx"))
	{
		// A text still being typed when the button takes the click is kept,
		// since its input is not drawn again to commit it
		if (isEditing(textId))
		{
			ioParameter.expression= s_edit.text;
			s_edit= EditBuffer();
		}
		ioParameter.bUseExpression= !bLit;
		bCommit= true;
	}
	if (bLit)
		ImGui::PopStyleColor();
	ImGui::SetItemTooltip("%s", locText(bLit ? "exprField.fxOnTooltip" : "exprField.fxOffTooltip"));
	ImGui::SameLine(0.f, spacing);
	ImGui::TextUnformatted(label);
	ImGui::SameLine();
	drawHelpMarker(table);

	if (bLit)
	{
		if (bCompileError)
			FaceTabWidgets::drawErrorText(compiled.errors.front());
		drawLiveBar(ioParameter, minValue, maxValue, valueFormat, bCompileError, live, fieldWidth);
	}
	ImGui::PopID();
	return bCommit;
}
