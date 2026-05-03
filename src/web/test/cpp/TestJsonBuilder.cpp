// Tests for src/web/src/json_builder.h.
//
// The motivating bug (caught in production by the trace-mix walker
// on a hierarchical real design): JsonBuilder used a fixed-size
// `bool need_comma_[16]` stack array. When the emitter exceeded 16
// nested levels it silently corrupted stack memory in release builds
// (the `assert(depth_ < kMaxDepth)` only fires in debug), producing
// malformed output like `"clocks": [, "sysclk"...` -- a leading
// comma in an array because the comma-state byte was clobbered.
// Switching `need_comma_` to a `std::vector<bool>` removed the cap
// and the silent-overflow class entirely. These tests pin that down.
#include "json_builder.h"
#include <gtest/gtest.h>

#include <boost/json.hpp>
#include <string>

namespace bj = boost::json;
using web::JsonBuilder;

namespace {

// Boost.json's default max_depth is 32, smaller than the depths we
// deliberately exercise here. Use a parser configured for very deep
// trees so the test failures we DO see are real malformed-JSON
// regressions, not the parser's own depth limit.
static bj::parse_options deepParseOpts()
{
  bj::parse_options opts;
  opts.max_depth = 256;
  return opts;
}

// Build a deeply-nested keyed-object structure and verify the output
// parses. Without dynamic depth, depth_ runs off the end of the
// fixed-size need_comma_ buffer at level 16+ and either asserts
// (debug) or corrupts surrounding stack memory (release).
TEST(JsonBuilderTest, DeeplyNestedObjectsParseValid)
{
  JsonBuilder b;
  constexpr int kDepth = 64;
  for (int i = 0; i < kDepth; ++i) {
    if (i == 0) {
      b.beginObject();
    } else {
      b.beginObject("child");
    }
    b.field("level", i);
  }
  for (int i = 0; i < kDepth; ++i) {
    b.endObject();
  }
  std::error_code ec;
  bj::value v = bj::parse(b.str(), ec, {}, deepParseOpts());
  ASSERT_FALSE(ec) << "deep-nested JSON failed to parse: " << ec.message()
                    << " — output was: " << b.str();
  // Walk to the deepest level and confirm the value survived intact.
  const bj::value* cur = &v;
  for (int i = 0; i < kDepth; ++i) {
    ASSERT_TRUE(cur->is_object()) << "depth " << i << " not an object";
    EXPECT_EQ(cur->as_object().at("level").as_int64(), i);
    if (i + 1 < kDepth) {
      cur = &cur->as_object().at("child");
    }
  }
}

// Same shape that broke production: deeply-nested arrays mixed with
// objects, with multiple values per array. The leading-comma symptom
// surfaced specifically on `clocks` arrays inside deeply-nested
// branch objects on the trace-mix tree.
TEST(JsonBuilderTest, DeeplyNestedArraysWithValuesNoLeadingComma)
{
  JsonBuilder b;
  constexpr int kDepth = 32;
  b.beginObject();
  // Open kDepth levels of {"branches": [{"clocks": ["a", "b"], "child": ...}]}
  for (int i = 0; i < kDepth; ++i) {
    b.beginArray("branches");
    b.beginObject();
    b.beginArray("clocks");
    b.value(std::string("a"));
    b.value(std::string("b"));
    b.endArray();
  }
  // Close them all.
  for (int i = 0; i < kDepth; ++i) {
    b.endObject();
    b.endArray();
  }
  b.endObject();
  const std::string& json = b.str();
  std::error_code ec;
  bj::value v = bj::parse(json, ec, {}, deepParseOpts());
  ASSERT_FALSE(ec) << "deep-nested array JSON failed to parse: "
                   << ec.message() << " — output was: " << json;
  // The pre-fix bug produced `"clocks": [, "a", "b"]` — a leading
  // comma inside the array. Boost.json's parser rejects that, so
  // the parse-success above is itself the regression check; this
  // additional walk verifies the actual value content is correct
  // at every depth.
  const bj::value* cur = &v;
  for (int i = 0; i < kDepth; ++i) {
    ASSERT_TRUE(cur->is_object()) << "depth " << i << " not an object";
    const auto& branches = cur->as_object().at("branches").as_array();
    ASSERT_EQ(branches.size(), 1u) << "depth " << i;
    const bj::value& branch_val = branches.front();
    ASSERT_TRUE(branch_val.is_object())
        << "depth " << i << " branch entry not an object";
    const auto& clocks = branch_val.as_object().at("clocks").as_array();
    ASSERT_EQ(clocks.size(), 2u) << "depth " << i << " clocks size";
    EXPECT_EQ(clocks[0].as_string(), "a") << "depth " << i;
    EXPECT_EQ(clocks[1].as_string(), "b") << "depth " << i;
    cur = &branch_val;
  }
}

// Mixed nesting (alternating arrays and objects) used to exhaust the
// fixed-depth `need_comma_` stack faster than pure-object nesting,
// because every array AND every object pushes a context. This test
// exercises that pattern beyond the old kMaxDepth=16.
TEST(JsonBuilderTest, AlternatingObjectArrayBeyondLegacyCap)
{
  JsonBuilder b;
  // 18 alternations = 36 push contexts, well past the legacy
  // kMaxDepth=16 cap.
  constexpr int kAlts = 18;
  b.beginObject();
  for (int i = 0; i < kAlts; ++i) {
    b.beginArray("a");
    b.beginObject();
  }
  b.field("leaf", 42);
  for (int i = 0; i < kAlts; ++i) {
    b.endObject();
    b.endArray();
  }
  b.endObject();

  std::error_code ec;
  bj::value v = bj::parse(b.str(), ec, {}, deepParseOpts());
  ASSERT_FALSE(ec) << "alternating-nesting JSON failed to parse: "
                   << ec.message() << " — output was: " << b.str();
  // Walk to the leaf and confirm the value survived intact.
  const bj::value* cur = &v;
  for (int i = 0; i < kAlts; ++i) {
    cur = &cur->as_object().at("a").as_array().front();
  }
  EXPECT_EQ(cur->as_object().at("leaf").as_int64(), 42);
}

// Sanity: small payloads (the common case) still produce the same
// canonical output. Catches an accidental break in the comma logic
// while we were swapping the storage representation.
TEST(JsonBuilderTest, ShallowOutputIsUnchanged)
{
  JsonBuilder b;
  b.beginObject();
  b.field("name", std::string("x"));
  b.beginArray("items");
  b.value(1);
  b.value(2);
  b.value(3);
  b.endArray();
  b.field("ok", true);
  b.endObject();
  EXPECT_EQ(b.str(),
            R"({"name": "x", "items": [1, 2, 3], "ok": true})");
}

// Empty containers — pre-fix this still worked, but the test catches
// regressions in the popContext / maybeComma reset logic.
TEST(JsonBuilderTest, EmptyContainersEmitCorrectly)
{
  JsonBuilder b;
  b.beginObject();
  b.beginArray("empty_arr");
  b.endArray();
  b.beginObject("empty_obj");
  b.endObject();
  b.endObject();
  EXPECT_EQ(b.str(), R"({"empty_arr": [], "empty_obj": {}})");
}

}  // namespace
