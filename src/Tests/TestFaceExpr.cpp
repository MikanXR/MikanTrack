#include "TestCommon.h"

#include "FaceExprCompiler.h"
#include "FaceExprEvaluator.h"
#include "FaceExprInputs.h"
#include "FaceExprParser.h"
#include "FaceProfile.h"
#include "FaceProfileLibrary.h"

// Tests for the face expression language: the parser, the compiler and its
// errors, every function, the time rule, the state slots across a recompile,
// the input sets, and the profile JSON.

namespace
{
const char* k_label= "test-faceexpr";

FaceProfile profileOf(const std::vector<std::string>& outputExprs,
					  const std::vector<std::pair<std::string, std::string>>& lets= {})
{
	FaceProfile profile;
	profile.id= "test";
	for (const auto& let : lets)
		profile.lets.push_back(FaceProfileLet{let.first, let.second});
	for (size_t index= 0; index < outputExprs.size(); ++index)
	{
		FaceProfileOutput output;
		output.targetKind= eFaceTargetKind::Morph;
		output.target= "out" + std::to_string(index);
		output.vrmExpressionFunction= outputExprs[index];
		profile.outputs.push_back(output);
	}
	return profile;
}

// One expression's value over the sources, or NaN when it fails to compile
float evalOnce(const std::string& expr, const FaceSources& sources= FaceSources())
{
	const FaceExprCompileResult result= compileFaceProfile(profileOf({expr}));
	if (result.program == nullptr)
		return NAN;
	FaceExprEvaluator evaluator;
	evaluator.setProgram(result.program);
	FaceExprState state;
	float value= 0.f;
	evaluator.evaluate(sources, 0.0, state, &value);
	return value;
}

bool near(float a, float b, float tolerance= 1e-5f)
{
	return fabsf(a - b) <= tolerance;
}

FaceSources phoneSources(const char* name, float value)
{
	FaceSources sources;
	sources.phone.present= true;
	sources.phone.tracked= true;
	sources.phone.blendshapes[arkitBlendshapeFromName(name)]= value;
	return sources;
}
} // namespace

static int runFaceExprTest(const TestArgs&)
{
	int failures= 0;
	auto check= [&](bool bCondition, const char* name) {
		if (bCondition)
		{
			MIKAN_LOG_INFO(k_label) << "PASS " << name;
		}
		else
		{
			MIKAN_LOG_ERROR(k_label) << "FAIL " << name;
			failures++;
		}
	};

	// (a) Precedence and associativity
	{
		check(near(evalOnce("1 + 2 * 3"), 7.f), "* binds tighter than +");
		check(near(evalOnce("(1 + 2) * 3"), 9.f), "parentheses group");
		check(near(evalOnce("8 - 3 - 2"), 3.f), "- is left associative");
		check(near(evalOnce("8 / 4 / 2"), 1.f), "/ is left associative");
		check(near(evalOnce("-2 * 3"), -6.f) && near(evalOnce("- -2"), 2.f), "unary minus binds tightest");
		check(near(evalOnce("1 + 1 > 1"), 1.f), "comparison binds looser than arithmetic");
		check(near(evalOnce("1 < 2 == 1"), 1.f), "== binds looser than <");
		check(near(evalOnce("1 || 0 && 0"), 1.f), "&& binds tighter than ||");
		check(near(evalOnce("!0 + 1"), 2.f), "! binds tighter than +");
		check(near(evalOnce("1.5e1"), 15.f) && near(evalOnce(".5"), 0.5f), "number forms");
	}

	// (b) Every operator and function
	{
		check(near(evalOnce("1 / 0"), 0.f), "division by zero is zero");
		check(near(evalOnce("2 <= 2") + evalOnce("3 >= 4") + evalOnce("1 != 1") + evalOnce("1 == 1"), 2.f),
			  "comparisons yield 0 or 1");
		check(near(evalOnce("0.6 && 0.4"), 0.f) && near(evalOnce("0.6 || 0.4"), 1.f) && near(evalOnce("!0.6"), 0.f),
			  "a value counts as true above 0.5");
		check(near(evalOnce("min(3, 1, 2)"), 1.f) && near(evalOnce("max(3, 1, 5, 2)"), 5.f), "min and max take any count");
		check(near(evalOnce("clamp(2, 0, 1)"), 1.f) && near(evalOnce("clamp(-1, 0, 1)"), 0.f), "clamp");
		check(near(evalOnce("abs(-0.25)"), 0.25f), "abs");
		check(near(evalOnce("pow(2, 3)"), 8.f) && near(evalOnce("pow(-1, 0.5)"), 0.f), "pow, NaN to zero");
		check(near(evalOnce("lerp(2, 4, 0.25)"), 2.5f), "lerp");
		check(near(evalOnce("remap(0.5, 0, 2)"), 0.25f) && near(evalOnce("remap(5, 0, 2)"), 1.f) &&
				  near(evalOnce("remap(0.5, 0, 1, 10, 20)"), 15.f),
			  "remap, three and five arguments, clamped to the input range");
		check(near(evalOnce("smoothstep(0, 1, 0.5)"), 0.5f) && near(evalOnce("smoothstep(0, 1, 0.25)"), 0.15625f),
			  "smoothstep");
		check(near(evalOnce("select(1, 2, 3)"), 2.f) && near(evalOnce("select(0, 2, 3)"), 3.f), "select");
	}

	// (c) Identifiers, compared without case in every namespace
	{
		const FaceSources jaw= phoneSources("jawOpen", 0.7f);
		check(near(evalOnce("phone.jawOpen", jaw), 0.7f) && near(evalOnce("PHONE.JAWOPEN", jaw), 0.7f),
			  "phone blendshapes without case");
		const FaceSources blink= phoneSources("eyeBlinkLeft", 0.3f);
		check(near(evalOnce("phone.eyeBlink_L", blink), 0.3f), "the phone's _L suffix spelling");
		check(near(evalOnce("Phone.Tracked", jaw), 1.f) && near(evalOnce("phone.present", jaw), 1.f),
			  "phone.present and phone.tracked");

		FaceSources mouth;
		mouth.viseme.present= true;
		mouth.viseme.weights[(int)eViseme::aa]= 0.4f;
		mouth.viseme.weights[(int)eViseme::PP]= 0.2f;
		check(near(evalOnce("viseme.AA", mouth), 0.4f) && near(evalOnce("viseme.pp", mouth), 0.2f) &&
				  near(evalOnce("VISEME.present", mouth), 1.f),
			  "visemes without case");

		FaceProfile profile= profileOf({"MouthOpen * 2"}, {{"mouthOpen", "phone.jawOpen"}});
		const FaceExprCompileResult result= compileFaceProfile(profile);
		check(result.program != nullptr, "a let is named without case");

		// Named in another case, a variable reads the same value as spelled
		// exactly
		FaceProfile cased= profileOf({"MOUTHOPEN * 2", "mouthOpen * 2"}, {{"mouthOpen", "phone.jawOpen"}});
		const FaceExprCompileResult casedResult= compileFaceProfile(cased);
		bool bCasedValue= false;
		if (casedResult.program != nullptr)
		{
			FaceExprEvaluator evaluator;
			evaluator.setProgram(casedResult.program);
			FaceExprState state;
			float values[2]= {-1.f, -1.f};
			evaluator.evaluate(phoneSources("jawOpen", 0.3f), 0.0, state, values);
			bCasedValue= near(values[0], 0.6f) && values[0] == values[1];
		}
		check(bCasedValue, "a variable named in another case evaluates as the exact spelling does");

		check(FaceExprInputs::fromName("phone.nope") < 0 && FaceExprInputs::fromName("webcam.jawOpen") < 0,
			  "an unknown name or namespace is not an input");
		bool bNamesRoundTrip= true;
		for (int slot= 0; slot < FaceExprInputs::k_count; ++slot)
			bNamesRoundTrip&= FaceExprInputs::fromName(FaceExprInputs::name(slot)) == slot;
		check(bNamesRoundTrip, "every slot's name resolves to it");
	}

	// (d) A source that is present but not tracked reads zero
	{
		FaceSources lost= phoneSources("jawOpen", 0.7f);
		lost.phone.tracked= false;
		check(near(evalOnce("phone.jawOpen", lost), 0.f) && near(evalOnce("phone.present", lost), 1.f) &&
				  near(evalOnce("phone.tracked", lost), 0.f),
			  "a lost phone face reads zero, present but not tracked");
		FaceSources stale;
		stale.viseme.weights[(int)eViseme::aa]= 0.9f;
		check(near(evalOnce("viseme.aa", stale), 0.f), "a stale mouth reads zero");
	}

	// (e) Lets: dependency order, cycles, errors with spans
	{
		FaceProfile ordered= profileOf({"c"}, {{"c", "b + 1"}, {"b", "a * 2"}, {"a", "3"}});
		FaceExprCompileResult result= compileFaceProfile(ordered);
		bool bValue= false;
		if (result.program != nullptr)
		{
			FaceExprEvaluator evaluator;
			evaluator.setProgram(result.program);
			FaceExprState state;
			float value= 0.f;
			evaluator.evaluate(FaceSources(), 0.0, state, &value);
			bValue= near(value, 7.f) && near(evaluator.getLetValue(1), 6.f);
		}
		check(bValue, "lets run in dependency order, not profile order");
		// c reads b reads a: a, then b, then c
		bool bOrder= false;
		if (result.program != nullptr)
		{
			const std::vector<int>& order= result.program->letOrder;
			const auto position= [&order](int letIndex) {
				return std::find(order.begin(), order.end(), letIndex) - order.begin();
			};
			bOrder= order.size() == 3 && position(2) < position(1) && position(1) < position(0);
		}
		check(bOrder, "letOrder lists a variable after the variables it reads");

		// The cycle is reported on the reference that closes it: a reads b,
		// and b's "a" closes the loop
		result= compileFaceProfile(profileOf({"1"}, {{"a", "1 + b"}, {"b", "2 * a"}}));
		bool bCycleSpan= false;
		for (const FaceExprCompileError& error : result.errors)
		{
			if (error.code == eFaceExprErrorCode::Cycle)
				bCycleSpan= error.bLet && error.index == 1 && !error.bOnName && error.span.begin == 4 && error.span.end == 5;
		}
		check(bCycleSpan, "a cycle error spans the reference that closes the cycle");

		// The name errors are on the name, spanning it
		const auto nameError= [](const FaceProfile& profile, eFaceExprErrorCode code) {
			const FaceExprCompileResult nameResult= compileFaceProfile(profile);
			for (const FaceExprCompileError& error : nameResult.errors)
			{
				if (error.code == code)
					return error.bLet && error.bOnName && error.span.begin == 0 &&
						   error.span.end == (int)profile.lets[(size_t)error.index].name.size();
			}
			return false;
		};
		check(nameError(profileOf({"1"}, {{"phone.x", "1"}}), eFaceExprErrorCode::InvalidVariableName) &&
				  nameError(profileOf({"1"}, {{"smooth", "1"}}), eFaceExprErrorCode::VariableIsFunctionName) &&
				  nameError(profileOf({"1"}, {{"a", "1"}, {"A", "2"}}), eFaceExprErrorCode::VariableDefinedTwice),
			  "the three variable name errors are marked on the name");

		FaceProfile cyclic= profileOf({"a"}, {{"a", "b + 1"}, {"b", "a"}});
		result= compileFaceProfile(cyclic);
		bool bCycle= result.program == nullptr && !result.errors.empty();
		for (const FaceExprCompileError& error : result.errors)
			bCycle&= error.bLet || error.message.find("cycle") != std::string::npos || error.message.find("error") != std::string::npos;
		bool bCycleNamed= false;
		for (const FaceExprCompileError& error : result.errors)
			bCycleNamed|= error.bLet && error.message.find("cycle") != std::string::npos;
		check(bCycle && bCycleNamed, "a let cycle is a compile error");
		FaceProfile selfCycle= profileOf({"1"}, {{"a", "a"}});
		check(compileFaceProfile(selfCycle).program == nullptr, "a let naming itself is a cycle");

		FaceProfile unknown= profileOf({"1", "phone.jawOpen + phone.nope"});
		result= compileFaceProfile(unknown);
		check(result.program == nullptr && result.errors.size() == 1 && !result.errors[0].bLet &&
				  result.errors[0].index == 1 && result.errors[0].span.begin == 16 && result.errors[0].span.end == 26,
			  "an unknown input is an error on its output, spanning the name");
		result= compileFaceProfile(profileOf({"nope * 2"}));
		check(result.program == nullptr && result.errors.size() == 1 && result.errors[0].span.begin == 0 &&
				  result.errors[0].span.end == 4,
			  "an unknown name is an error spanning it");
		result= compileFaceProfile(profileOf({"wobble(1)"}));
		check(result.program == nullptr && result.errors.size() == 1 && result.errors[0].span.end == 6,
			  "an unknown function is an error spanning its name");
		result= compileFaceProfile(profileOf({"clamp(1, 2)"}));
		check(result.program == nullptr && result.errors.size() == 1, "a wrong argument count is an error");
		result= compileFaceProfile(profileOf({"remap(1, 2, 3, 4)"}));
		check(result.program == nullptr, "remap takes three or five arguments, not four");
		result= compileFaceProfile(profileOf({"1 +", "(1", "1 $ 2"}));
		check(result.program == nullptr && result.errors.size() == 3 && result.errors[2].span.begin == 2,
			  "syntax errors are reported per output with their spans");
		result= compileFaceProfile(profileOf({"1"}, {{"a", "1"}, {"A", "2"}}));
		check(result.program == nullptr && result.errors.size() == 1 && result.errors[0].bLet &&
				  result.errors[0].index == 1,
			  "a let named twice, without case, is an error");
		result= compileFaceProfile(profileOf({"1"}, {{"smooth", "1"}}));
		check(result.program == nullptr, "a let named after a function is an error");
		result= compileFaceProfile(profileOf({"1"}, {{"phone.x", "1"}}));
		check(result.program == nullptr, "a dotted let name is an error");
	}

	// (f) smooth converges at its tau under varying dt
	{
		const FaceExprCompileResult result= compileFaceProfile(profileOf({"smooth(phone.jawOpen, 0.1)"}));
		bool bConverged= false;
		bool bTauMatched= false;
		if (result.program != nullptr)
		{
			FaceExprEvaluator evaluator;
			evaluator.setProgram(result.program);
			FaceExprState state;
			float value= 0.f;
			evaluator.evaluate(phoneSources("jawOpen", 0.f), 0.0, state, &value);
			// A step to 1 with uneven frame spacing: after one tau the output
			// is 1 - 1/e whatever the spacing was
			const double steps[]= {0.004, 0.021, 0.009, 0.033, 0.016, 0.017};
			double time= 0.0;
			const FaceSources up= phoneSources("jawOpen", 1.f);
			while (time < 0.1 - 1e-9)
			{
				for (double step : steps)
				{
					if (time + step > 0.1)
						step= 0.1 - time;
					time+= step;
					evaluator.evaluate(up, time, state, &value);
					if (time >= 0.1 - 1e-9)
						break;
				}
			}
			bTauMatched= near(value, 1.f - expf(-1.f), 1e-3f);
			for (int frame= 0; frame < 200; ++frame)
			{
				time+= 0.016;
				evaluator.evaluate(up, time, state, &value);
			}
			bConverged= near(value, 1.f, 1e-4f);
		}
		check(bTauMatched, "smooth reaches 1 - 1/e after one tau under uneven dt");
		check(bConverged, "smooth converges to its input");
	}

	// (g) The time rule
	{
		const FaceExprCompileResult result= compileFaceProfile(profileOf({"smooth(phone.jawOpen, 0.1)"}));
		bool bRegressionStill= false;
		bool bCapReset= false;
		bool bExplicitReset= false;
		bool bForwardCapped= false;
		if (result.program != nullptr)
		{
			FaceExprEvaluator evaluator;
			evaluator.setProgram(result.program);
			FaceExprState state;
			float value= 0.f;
			evaluator.evaluate(phoneSources("jawOpen", 0.f), 10.0, state, &value);
			evaluator.evaluate(phoneSources("jawOpen", 1.f), 9.995, state, &value);
			bRegressionStill= value == 0.f && state.lastTimeSeconds == 10.0;
			// The next forward frame steps from the clock that held, 10.0
			evaluator.evaluate(phoneSources("jawOpen", 1.f), 10.1, state, &value);
			bRegressionStill&= near(value, 1.f - expf(-1.f), 1e-3f);

			evaluator.evaluate(phoneSources("jawOpen", 0.25f), 8.0, state, &value);
			bCapReset= value == 0.25f;

			state.reset();
			evaluator.evaluate(phoneSources("jawOpen", 0.75f), 8.01, state, &value);
			bExplicitReset= value == 0.75f;

			// Forward by an hour steps one second, 10 taus
			evaluator.evaluate(phoneSources("jawOpen", 0.f), 3608.0, state, &value);
			bForwardCapped= near(value, 0.75f * expf(-10.f), 1e-5f);
		}
		check(bRegressionStill, "a small backwards step advances nothing and keeps the clock");
		check(bCapReset, "a backwards jump past the cap reinitializes");
		check(bExplicitReset, "an explicit reset reinitializes");
		check(bForwardCapped, "a forward jump steps at most the cap");
	}

	// (h) hysteresis latches
	{
		const FaceExprCompileResult result= compileFaceProfile(profileOf({"hysteresis(phone.jawOpen, 0.6, 0.3)"}));
		bool bLatching= false;
		if (result.program != nullptr)
		{
			FaceExprEvaluator evaluator;
			evaluator.setProgram(result.program);
			FaceExprState state;
			const float inputs[]= {0.5f, 0.6f, 0.4f, 0.31f, 0.3f, 0.5f, 0.7f};
			const float expected[]= {0.f, 1.f, 1.f, 1.f, 0.f, 0.f, 1.f};
			bLatching= true;
			for (size_t index= 0; index < 7; ++index)
			{
				float value= -1.f;
				evaluator.evaluate(phoneSources("jawOpen", inputs[index]), 0.01 * (double)index, state, &value);
				bLatching&= value == expected[index];
			}
		}
		check(bLatching, "hysteresis turns on at on, off at off, and holds between");
	}

	// (i) State per call site, and across a recompile
	{
		FaceProfile profile= profileOf({"smooth(phone.jawOpen, 0.1) + 0 * smooth(phone.jawOpen, 1000)",
										"smooth(phone.jawOpen, 0.1)"});
		FaceExprCompileResult result= compileFaceProfile(profile);
		check(result.program != nullptr && result.program->stateSlotKeys.size() == 3,
			  "each stateful call site gets its own slot");

		bool bSeparate= false;
		bool bCarried= false;
		bool bEditedReset= false;
		if (result.program != nullptr)
		{
			FaceExprEvaluator evaluator;
			evaluator.setProgram(result.program);
			FaceExprState state;
			float values[2]= {};
			evaluator.evaluate(phoneSources("jawOpen", 0.f), 0.0, state, values);
			evaluator.evaluate(phoneSources("jawOpen", 1.f), 0.1, state, values);
			// The slow call sits beside the fast one in the same output
			bSeparate= state.slots.size() == 3 && near(state.slots[0].value, 1.f - expf(-1.f), 1e-3f) &&
					   state.slots[1].value < 1e-3f;

			// Edit the second output: the first keeps its smoothing, the
			// second starts over from its input
			profile.outputs[1].vrmExpressionFunction= "smooth(phone.jawOpen, 0.2)";
			const FaceExprCompileResult edited= compileFaceProfile(profile);
			if (edited.program != nullptr)
			{
				evaluator.setProgram(edited.program);
				evaluator.evaluate(phoneSources("jawOpen", 1.f), 0.1, state, values);
				bCarried= near(values[0], 1.f - expf(-1.f), 1e-3f);
				bEditedReset= values[1] == 1.f;
			}
		}
		check(bSeparate, "two call sites in one expression keep separate state");
		check(bCarried, "an unchanged output keeps its state across a recompile");
		check(bEditedReset, "an edited output starts its state over");
	}

	// (j) Input sets
	{
		FaceProfile profile= profileOf({"open + phone.eyeBlinkLeft", "viseme.aa"},
									   {{"open", "max(phone.jawOpen, base)"}, {"base", "viseme.present"}});
		const FaceExprCompileResult result= compileFaceProfile(profile);
		bool bSets= false;
		if (result.program != nullptr)
		{
			const FaceExprProgram::Expression& first= result.program->outputs[0];
			const int jaw= FaceExprInputs::fromName("phone.jawOpen");
			const int blink= FaceExprInputs::fromName("phone.eyeBlinkLeft");
			const int present= FaceExprInputs::fromName("viseme.present");
			std::vector<int> expectedInputs= {jaw, blink, present};
			std::sort(expectedInputs.begin(), expectedInputs.end());
			bSets= first.directInputs == std::vector<int>{blink} && first.inputs == expectedInputs &&
				   first.directLets == std::vector<int>{0} && first.lets == (std::vector<int>{0, 1}) &&
				   result.program->outputs[1].inputs == std::vector<int>{FaceExprInputs::fromName("viseme.aa")} &&
				   result.program->lets[0].inputs == (std::vector<int>{std::min(jaw, present), std::max(jaw, present)});
		}
		check(bSets, "each output and let reports its direct and transitive inputs and lets");
	}

	// (k) The profile JSON round trip keeps the id
	{
		FaceProfile profile;
		profile.id= makeFaceProfileId();
		profile.name= "Round trip";
		profile.lets.push_back(FaceProfileLet{"open", "phone.jawOpen"});
		FaceProfileOutput expression;
		expression.targetKind= eFaceTargetKind::Expression;
		expression.target= "A";
		expression.vrmExpressionFunction= "open";
		profile.outputs.push_back(expression);
		FaceProfileOutput morph;
		morph.targetKind= eFaceTargetKind::Morph;
		morph.target= "Fcl_MTH_A";
		morph.mesh= "Face";
		morph.vrmExpressionFunction= "smooth(viseme.aa, 0.05)";
		profile.outputs.push_back(morph);

		FaceProfile loaded;
		std::string error;
		const bool bParsed= faceProfileFromJson(faceProfileToJson(profile), loaded, error);
		check(bParsed && loaded.id == profile.id && loaded.name == profile.name && loaded.lets.size() == 1 &&
				  loaded.lets[0].name == "open" && loaded.lets[0].expr == "phone.jawOpen" && loaded.outputs.size() == 2 &&
				  loaded.outputs[0].targetKind == eFaceTargetKind::Expression && loaded.outputs[0].mesh.empty() &&
				  loaded.outputs[1].targetKind == eFaceTargetKind::Morph && loaded.outputs[1].mesh == "Face" &&
				  loaded.outputs[1].vrmExpressionFunction == morph.vrmExpressionFunction,
			  "a profile round-trips through JSON with its id");
		check(profile.id.size() == 36 && profile.id[14] == '4' && profile.id != makeFaceProfileId(),
			  "a minted id is a fresh version 4 UUID");
		check(!faceProfileFromJson("{\"name\": \"x\"}", loaded, error), "a profile without an id does not load");

		const std::string written= faceProfileToJson(profile);
		check(written.find("\"variables\"") != std::string::npos && written.find("\"lets\"") == std::string::npos,
			  "the variables are written under \"variables\"");
		const std::string legacy= "{\"id\": \"legacy\", \"name\": \"Old\", \"lets\": [{\"name\": \"open\", \"expr\": "
								  "\"phone.jawOpen\"}], \"outputs\": []}";
		check(faceProfileFromJson(legacy, loaded, error) && loaded.id == "legacy" && loaded.lets.size() == 1 &&
				  loaded.lets[0].name == "open" && loaded.lets[0].expr == "phone.jawOpen",
			  "a file with the old \"lets\" key loads its variables");

		// The variable alone keeps "expr"
		const size_t firstExpr= written.find("\"expr\"");
		check(written.find("\"vrmExpressionFunction\": \"open\"") != std::string::npos &&
				  written.find("\"vrmExpressionFunction\": \"smooth(viseme.aa, 0.05)\"") != std::string::npos &&
				  firstExpr != std::string::npos && written.find("\"expr\"", firstExpr + 1) == std::string::npos,
			  "a binding's function is written under \"vrmExpressionFunction\"");
		const std::string legacyBinding= "{\"id\": \"legacy\", \"name\": \"Old\", \"outputs\": [{\"targetKind\": "
										 "\"expression\", \"target\": \"A\", \"expr\": \"viseme.aa\"}]}";
		check(faceProfileFromJson(legacyBinding, loaded, error) && loaded.outputs.size() == 1 &&
				  loaded.outputs[0].target == "A" && loaded.outputs[0].vrmExpressionFunction == "viseme.aa",
			  "a binding with the old \"expr\" key loads its function");
	}

	// (l) The library: one file per id, rename keeps the id, a duplicate
	// gets a fresh one, an unreadable file is skipped
	{
		const std::filesystem::path folder= std::filesystem::temp_directory_path() / "mikan-test-faceprofiles";
		std::error_code error;
		std::filesystem::remove_all(folder, error);
		FaceProfileLibrary library(folder);
		library.refresh();
		const bool bEmpty= library.getProfiles().empty();

		const std::string firstId= library.create("Zeta");
		FaceProfile edited= *library.find(firstId);
		FaceProfileOutput output;
		output.target= "A";
		output.vrmExpressionFunction= "viseme.aa";
		edited.outputs.push_back(output);
		library.save(edited);
		const std::string copyId= library.duplicate(edited, "Alpha");
		library.rename(firstId, "Omega");
		{
			std::ofstream junk(folder / "broken.json");
			junk << "{ not json";
		}

		FaceProfileLibrary reread(folder);
		reread.refresh();
		const FaceProfile* first= reread.find(firstId);
		const FaceProfile* copy= reread.find(copyId);
		check(bEmpty && reread.getProfiles().size() == 2 && first != nullptr && copy != nullptr &&
				  std::filesystem::is_regular_file(folder / (firstId + ".json")),
			  "the library is one <id>.json per profile and skips an unreadable file");
		check(first != nullptr && first->name == "Omega" && first->outputs.size() == 1,
			  "a rename rewrites the name and keeps the id and the content");
		check(copy != nullptr && copyId != firstId && copy->name == "Alpha" && copy->outputs.size() == 1,
			  "a duplicate is a copy under a fresh id");
		check(reread.getProfiles().size() == 2 && reread.getProfiles()[0].name == "Alpha",
			  "the library lists by name");
		check(reread.remove(copyId) && reread.find(copyId) == nullptr &&
				  !std::filesystem::exists(folder / (copyId + ".json")),
			  "remove deletes the file");
		FaceProfile hostile;
		hostile.id= "../escape";
		check(!reread.save(hostile) && !FaceProfileLibrary::isValidId("a/b") && FaceProfileLibrary::isValidId(firstId),
			  "an id that is not a plain UUID-like name never names a file");
		std::filesystem::remove_all(folder, error);
	}

	if (failures == 0)
		MIKAN_LOG_INFO(k_label) << "All face expression tests passed";
	return failures == 0 ? 0 : 1;
}

MIKAN_REGISTER_TEST("--test-faceexpr",
					"face expressions: precedence, functions, names, variables, smooth, the time rule, hysteresis, state "
					"slots across a recompile, input sets, profile JSON, the library",
					eTestCategory::SelfTest, runFaceExprTest);
