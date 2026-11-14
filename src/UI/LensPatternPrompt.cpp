#include "LensPatternPrompt.h"

#include "imgui.h"

#include "opencv2/core/mat.hpp"

#include "AppConfig.h"
#include "GlTexture.h"
#include "LocText.h"
#include "PatternExport.h"

LensPatternPrompt::LensPatternPrompt()= default;
LensPatternPrompt::~LensPatternPrompt()= default;

void LensPatternPrompt::drawPatternChoice(eIntrinsicsPattern& pattern)
{
	int choice= (int)pattern;
	ImGui::RadioButton(locLabel("intrinsicsWizard.patternPhoneRadio"), &choice,
					   (int)eIntrinsicsPattern::PhoneChessboard);
	ImGui::RadioButton(locLabel("intrinsicsWizard.patternPrintedRadio"), &choice,
					   (int)eIntrinsicsPattern::PrintedCharuco);
	pattern= (eIntrinsicsPattern)choice;
}

void LensPatternPrompt::drawPhoneInstructions()
{
	if (!m_bQrCodeBuilt)
	{
		m_bQrCodeBuilt= true;
		const cv::Mat qrImage= renderQrCodeImage(k_phonePatternUrl, 6);
		if (!qrImage.empty())
		{
			m_qrTexture= std::make_unique<GlTexture>();
			// The texture shows the frame uploaded before the newest one, so
			// a still image goes in twice
			m_qrTexture->uploadBGR(qrImage);
			m_qrTexture->uploadBGR(qrImage);
		}
	}

	ImGui::TextWrapped("%s", locText("intrinsicsWizard.phoneInstructions"));
	ImGui::Spacing();

	if (m_qrTexture != nullptr && m_qrTexture->getIsValid())
	{
		ImGui::Image((ImTextureID)(intptr_t)m_qrTexture->getGlTextureId(),
					 ImVec2((float)m_qrTexture->getWidth(), (float)m_qrTexture->getHeight()));
	}
	ImGui::TextUnformatted(k_phonePatternUrl);
	ImGui::Spacing();

	for (const char* hintKey : {"intrinsicsWizard.phoneHintDistance", "intrinsicsWizard.phoneHintGlare",
								"intrinsicsWizard.phoneHintSteady"})
	{
		ImGui::Bullet();
		ImGui::TextWrapped("%s", locText(hintKey));
	}
	ImGui::Spacing();

	if (ImGui::Button(locLabel("intrinsicsWizard.exportPhonePatternButton")))
		exportAndOpenPhoneChessboard();
	ImGui::SetItemTooltip("%s", locText("intrinsicsWizard.exportPhonePatternTooltip"));
}
