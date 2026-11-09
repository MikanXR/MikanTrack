#include "FaceExprParser.h"

namespace
{
constexpr int k_prefixBindingPower= 7;

// The binding power of a binary operator; 0 for a token that is not one
int infixBindingPower(const FaceExprToken& token)
{
	if (token.kind != eFaceExprToken::Operator)
		return 0;
	const std::string& op= token.text;
	if (op == "||")
		return 1;
	if (op == "&&")
		return 2;
	if (op == "==" || op == "!=")
		return 3;
	if (op == "<" || op == "<=" || op == ">" || op == ">=")
		return 4;
	if (op == "+" || op == "-")
		return 5;
	if (op == "*" || op == "/")
		return 6;
	return 0;
}

class Parser
{
public:
	Parser(const std::vector<FaceExprToken>& tokens, FaceExprError& error)
		: m_tokens(tokens)
		, m_error(error)
	{}

	std::unique_ptr<FaceExprNode> parseAll()
	{
		std::unique_ptr<FaceExprNode> root= parseExpression(0);
		if (root == nullptr)
			return nullptr;
		if (peek().kind != eFaceExprToken::End)
			return failUnexpected(peek());
		return root;
	}

private:
	const FaceExprToken& peek() const { return m_tokens[m_position]; }
	const FaceExprToken& next() { return m_tokens[m_position++]; }

	std::unique_ptr<FaceExprNode> fail(eFaceExprErrorCode code, const FaceExprSpan& span)
	{
		m_error.code= code;
		m_error.args.clear();
		m_error.span= span;
		switch (code)
		{
		case eFaceExprErrorCode::ExpressionEndsEarly:
			m_error.message= "expression ends early";
			break;
		case eFaceExprErrorCode::ExpectedCloseParen:
			m_error.message= "expected ')'";
			break;
		case eFaceExprErrorCode::ExpectedCommaOrParen:
			m_error.message= "expected ',' or ')'";
			break;
		default:
			m_error.message= "syntax error";
			break;
		}
		return nullptr;
	}

	std::unique_ptr<FaceExprNode> failUnexpected(const FaceExprToken& token)
	{
		m_error.code= eFaceExprErrorCode::UnexpectedToken;
		m_error.args= {token.text};
		m_error.span= token.span;
		m_error.message= "unexpected '" + token.text + "'";
		return nullptr;
	}

	std::unique_ptr<FaceExprNode> parseExpression(int minBindingPower)
	{
		std::unique_ptr<FaceExprNode> left= parsePrefix();
		if (left == nullptr)
			return nullptr;

		for (;;)
		{
			const FaceExprToken& op= peek();
			const int bindingPower= infixBindingPower(op);
			if (bindingPower == 0 || bindingPower <= minBindingPower)
				break;
			next();
			// Left associative: the right side binds tighter than this level
			std::unique_ptr<FaceExprNode> right= parseExpression(bindingPower);
			if (right == nullptr)
				return nullptr;

			auto binary= std::make_unique<FaceExprNode>();
			binary->kind= eFaceExprNode::Binary;
			binary->text= op.text;
			binary->span= FaceExprSpan{left->span.begin, right->span.end};
			binary->children.push_back(std::move(left));
			binary->children.push_back(std::move(right));
			left= std::move(binary);
		}
		return left;
	}

	std::unique_ptr<FaceExprNode> parsePrefix()
	{
		const FaceExprToken& token= next();
		switch (token.kind)
		{
		case eFaceExprToken::Number:
		{
			auto node= std::make_unique<FaceExprNode>();
			node->kind= eFaceExprNode::Number;
			node->number= token.number;
			node->text= token.text;
			node->span= token.span;
			return node;
		}
		case eFaceExprToken::Identifier:
		{
			if (peek().kind == eFaceExprToken::LeftParen)
				return parseCall(token);
			auto node= std::make_unique<FaceExprNode>();
			node->kind= eFaceExprNode::Identifier;
			node->text= token.text;
			node->span= token.span;
			return node;
		}
		case eFaceExprToken::Operator:
		{
			if (token.text != "-" && token.text != "!")
				return failUnexpected(token);
			std::unique_ptr<FaceExprNode> operand= parseExpression(k_prefixBindingPower - 1);
			if (operand == nullptr)
				return nullptr;
			auto node= std::make_unique<FaceExprNode>();
			node->kind= eFaceExprNode::Unary;
			node->text= token.text;
			node->span= FaceExprSpan{token.span.begin, operand->span.end};
			node->children.push_back(std::move(operand));
			return node;
		}
		case eFaceExprToken::LeftParen:
		{
			std::unique_ptr<FaceExprNode> inner= parseExpression(0);
			if (inner == nullptr)
				return nullptr;
			if (peek().kind != eFaceExprToken::RightParen)
				return fail(eFaceExprErrorCode::ExpectedCloseParen, peek().span);
			const FaceExprToken& close= next();
			inner->span= FaceExprSpan{token.span.begin, close.span.end};
			return inner;
		}
		case eFaceExprToken::End:
			return fail(eFaceExprErrorCode::ExpressionEndsEarly, token.span);
		default:
			return failUnexpected(token);
		}
	}

	std::unique_ptr<FaceExprNode> parseCall(const FaceExprToken& name)
	{
		next(); // (
		auto call= std::make_unique<FaceExprNode>();
		call->kind= eFaceExprNode::Call;
		call->text= name.text;
		if (peek().kind != eFaceExprToken::RightParen)
		{
			for (;;)
			{
				std::unique_ptr<FaceExprNode> argument= parseExpression(0);
				if (argument == nullptr)
					return nullptr;
				call->children.push_back(std::move(argument));
				if (peek().kind == eFaceExprToken::Comma)
				{
					next();
					continue;
				}
				break;
			}
		}
		if (peek().kind != eFaceExprToken::RightParen)
			return fail(eFaceExprErrorCode::ExpectedCommaOrParen, peek().span);
		const FaceExprToken& close= next();
		call->span= FaceExprSpan{name.span.begin, close.span.end};
		return call;
	}

	const std::vector<FaceExprToken>& m_tokens;
	FaceExprError& m_error;
	size_t m_position= 0;
};
} // namespace

std::unique_ptr<FaceExprNode> parseFaceExpr(const std::string& text, FaceExprError& outError)
{
	std::vector<FaceExprToken> tokens;
	if (!lexFaceExpr(text, tokens, outError))
		return nullptr;
	Parser parser(tokens, outError);
	return parser.parseAll();
}
