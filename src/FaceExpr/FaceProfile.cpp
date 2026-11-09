#include "FaceProfile.h"

#include <cstdio>
#include <random>

#include "nlohmann/json.hpp"

using json= nlohmann::json;

const char* faceTargetKindName(eFaceTargetKind kind)
{
	return kind == eFaceTargetKind::Morph ? "morph" : "expression";
}

std::string makeFaceProfileId()
{
	std::random_device device;
	std::mt19937_64 generator(((uint64_t)device() << 32) ^ device());
	std::uniform_int_distribution<uint32_t> byteDistribution(0, 255);
	unsigned char bytes[16];
	for (unsigned char& byte : bytes)
		byte= (unsigned char)byteDistribution(generator);
	// Version 4, variant 10
	bytes[6]= (unsigned char)((bytes[6] & 0x0F) | 0x40);
	bytes[8]= (unsigned char)((bytes[8] & 0x3F) | 0x80);

	char text[37];
	snprintf(text, sizeof(text), "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x", bytes[0],
			 bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7], bytes[8], bytes[9], bytes[10],
			 bytes[11], bytes[12], bytes[13], bytes[14], bytes[15]);
	return text;
}

std::string faceProfileToJson(const FaceProfile& profile)
{
	json j;
	j["version"]= FaceProfile::k_version;
	j["id"]= profile.id;
	j["name"]= profile.name;

	json variables= json::array();
	for (const FaceProfileLet& let : profile.lets)
		variables.push_back({{"name", let.name}, {"expr", let.expr}});
	j["variables"]= variables;

	json outputs= json::array();
	for (const FaceProfileOutput& output : profile.outputs)
	{
		json jo= {{"targetKind", faceTargetKindName(output.targetKind)}, {"target", output.target},
				   {"vrmExpressionFunction", output.vrmExpressionFunction}};
		if (!output.mesh.empty())
			jo["mesh"]= output.mesh;
		outputs.push_back(jo);
	}
	j["outputs"]= outputs;
	return j.dump(2);
}

bool faceProfileFromJson(const std::string& text, FaceProfile& outProfile, std::string& outError)
{
	FaceProfile profile;
	try
	{
		const json j= json::parse(text);
		if (!j.is_object())
		{
			outError= "not a JSON object";
			return false;
		}
		profile.id= j.value("id", "");
		profile.name= j.value("name", "");
		if (profile.id.empty())
		{
			outError= "the profile has no id";
			return false;
		}

		// Files written before the rename carry "lets"
		const json variables= j.contains("variables") ? j["variables"] : j.value("lets", json::array());
		for (const json& jl : variables)
		{
			if (!jl.is_object())
				continue;
			profile.lets.push_back(FaceProfileLet{jl.value("name", ""), jl.value("expr", "")});
		}

		const json& outputs= j.value("outputs", json::array());
		for (const json& jo : outputs)
		{
			if (!jo.is_object())
				continue;
			FaceProfileOutput output;
			output.targetKind= jo.value("targetKind", "expression") == "morph" ? eFaceTargetKind::Morph
																			   : eFaceTargetKind::Expression;
			output.target= jo.value("target", "");
			output.mesh= jo.value("mesh", "");
			// Files written before the rename carry "expr"
			output.vrmExpressionFunction=
				jo.contains("vrmExpressionFunction") ? jo.value("vrmExpressionFunction", "") : jo.value("expr", "");
			profile.outputs.push_back(output);
		}
	}
	catch (const std::exception& exception)
	{
		outError= exception.what();
		return false;
	}

	outProfile= profile;
	return true;
}
