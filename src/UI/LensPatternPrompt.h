#pragma once

#include <memory>

class GlTexture;
enum class eIntrinsicsPattern;

// The lens calibration pattern choice, shared by the intrinsics wizard and
// the guided setup: the two radio buttons, and for the phone the QR code of
// the page that shows the chessboard, its address, and the hints.
class LensPatternPrompt
{
public:
	LensPatternPrompt();
	~LensPatternPrompt();

	// The two radio buttons, writing the choice into pattern
	void drawPatternChoice(eIntrinsicsPattern& pattern);
	// What the phone path needs in front of the user before capture
	void drawPhoneInstructions();

private:
	// Built on first draw, since it needs a current GL context
	std::unique_ptr<GlTexture> m_qrTexture;
	bool m_bQrCodeBuilt= false;
};
