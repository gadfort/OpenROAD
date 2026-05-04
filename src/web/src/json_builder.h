// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

#pragma once

// TRANSITIONAL SHIM. Upstream master converted JSON I/O to boost::json
// directly (commit 194c489c2d, "[web] Convert JSON I/O from hand-rolled
// code to Boost.JSON") and removed this header. The SDC and CDC handlers
// in this branch still have ~900 callsites of the streaming JsonBuilder
// API; this shim keeps that API working by buffering writes into a
// `boost::json::value` tree and serializing on `str()`.
//
// The shim should go away once the SDC/CDC handlers are migrated to the
// upstream idiom (build `boost::json::object` directly, `boost::json::
// serialize` at the response edge). Tracking entry: see TODO in
// plans/cdc-visualizer-plan.md.

#include <boost/json/array.hpp>
#include <boost/json/object.hpp>
#include <boost/json/serialize.hpp>
#include <boost/json/value.hpp>
#include <cassert>
#include <string>
#include <utility>
#include <vector>

namespace web {

class JsonBuilder
{
 public:
  JsonBuilder() = default;

  // -- Containers --

  void beginObject() { openContainer(boost::json::object{}); }

  void beginObject(const char* key)
  {
    pending_key_ = key;
    has_pending_key_ = true;
    openContainer(boost::json::object{});
  }
  void beginObject(const std::string& key) { beginObject(key.c_str()); }

  void endObject() { closeContainer(); }

  void beginArray() { openContainer(boost::json::array{}); }

  void beginArray(const char* key)
  {
    pending_key_ = key;
    has_pending_key_ = true;
    openContainer(boost::json::array{});
  }
  void beginArray(const std::string& key) { beginArray(key.c_str()); }

  void endArray() { closeContainer(); }

  // -- Named fields (inside objects) --

  void field(const char* key, const std::string& val)
  {
    insertIntoParent(key, boost::json::value(val));
  }
  void field(const std::string& key, const std::string& val)
  {
    field(key.c_str(), val);
  }
  void field(const char* key, const char* val)
  {
    insertIntoParent(key, boost::json::value(val));
  }
  void field(const char* key, int val)
  {
    insertIntoParent(key, boost::json::value(val));
  }
  void field(const char* key, float val)
  {
    insertIntoParent(key, boost::json::value(val));
  }
  void field(const char* key, double val)
  {
    insertIntoParent(key, boost::json::value(val));
  }
  void field(const char* key, bool val)
  {
    insertIntoParent(key, boost::json::value(val));
  }
  void field(const std::string& key, const char* val)
  {
    field(key.c_str(), val);
  }
  void field(const std::string& key, int val) { field(key.c_str(), val); }
  void field(const std::string& key, float val) { field(key.c_str(), val); }
  void field(const std::string& key, double val) { field(key.c_str(), val); }
  void field(const std::string& key, bool val) { field(key.c_str(), val); }

  // -- Unnamed values (inside arrays) --

  void value(const std::string& val)
  {
    insertIntoArray(boost::json::value(val));
  }
  void value(const char* val) { insertIntoArray(boost::json::value(val)); }
  void value(int val) { insertIntoArray(boost::json::value(val)); }
  void value(float val) { insertIntoArray(boost::json::value(val)); }
  void value(double val) { insertIntoArray(boost::json::value(val)); }
  void value(bool val) { insertIntoArray(boost::json::value(val)); }

  void nullField(const char* key)
  {
    insertIntoParent(key, boost::json::value(nullptr));
  }
  void nullField(const std::string& key) { nullField(key.c_str()); }

  void nullValue() { insertIntoArray(boost::json::value(nullptr)); }

  // -- Output --

  std::string str() const
  {
    finalize();
    return boost::json::serialize(serialized_);
  }

 private:
  // Each stack frame is the container currently being built. Storing
  // by value (not by pointer into a parent's storage) avoids the
  // pointer-invalidation problem that arises when boost::json::array
  // reallocates its backing on push_back: we only insert into the
  // parent when the frame is popped, so every operation sees a
  // stable address for the current container.
  struct Frame
  {
    boost::json::value container;
    std::string key;       // key to use when inserting into parent object
    bool keyed = false;    // false → parent is an array (or this is root)
  };

  // Open a new container frame on the stack. If a pending key was
  // queued by `beginObject(key)` / `beginArray(key)`, attach it now.
  void openContainer(boost::json::value v)
  {
    Frame f;
    f.container = std::move(v);
    if (has_pending_key_) {
      f.key = std::move(pending_key_);
      f.keyed = true;
      pending_key_.clear();
      has_pending_key_ = false;
    }
    stack_.push_back(std::move(f));
  }

  // Pop the top frame and insert it into its parent. If we're closing
  // the root frame, store it as the serialization root.
  void closeContainer()
  {
    assert(!stack_.empty());
    Frame f = std::move(stack_.back());
    stack_.pop_back();
    placeFrame(std::move(f));
  }

  // Place a leaf value into the parent. `key` is the field name when
  // the parent is an object.
  void insertIntoParent(const char* key, boost::json::value v)
  {
    Frame f;
    f.container = std::move(v);
    f.key = key;
    f.keyed = true;
    placeFrame(std::move(f));
  }

  // Place a value into the parent array.
  void insertIntoArray(boost::json::value v)
  {
    Frame f;
    f.container = std::move(v);
    placeFrame(std::move(f));
  }

  void placeFrame(Frame f)
  {
    if (stack_.empty()) {
      root_ = std::move(f.container);
      root_set_ = true;
      return;
    }
    auto& parent = stack_.back().container;
    if (parent.is_object()) {
      assert(f.keyed);
      parent.as_object()[f.key] = std::move(f.container);
    } else if (parent.is_array()) {
      parent.as_array().push_back(std::move(f.container));
    } else {
      assert(false && "JsonBuilder: parent is not a container");
    }
  }

  // Snapshot the current state for serialization. If the user never
  // explicitly closed the root container (e.g. their last call was
  // `b.beginObject(); ...` and they invoked `str()` without
  // `b.endObject()`), recover gracefully by treating the top frame
  // as the root. This matches the streaming-builder semantics where
  // `str()` was usable mid-construction.
  void finalize() const
  {
    if (root_set_) {
      serialized_ = root_;
      return;
    }
    if (!stack_.empty()) {
      serialized_ = stack_.front().container;
      return;
    }
    serialized_ = boost::json::value(nullptr);
  }

  std::vector<Frame> stack_;
  boost::json::value root_;
  bool root_set_ = false;
  std::string pending_key_;
  bool has_pending_key_ = false;
  // Snapshot for str(); must remain valid for the lifetime of the
  // returned reference if any caller stores `str()` by ref.
  mutable boost::json::value serialized_;
};

}  // namespace web
