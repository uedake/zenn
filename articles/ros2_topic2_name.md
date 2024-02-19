---
title: "ROS2を深く理解する：トピック編２　トピック名"
emoji: "📑"
type: "tech"
topics:
  - "ros2"
  - "robot"
  - "robotics"
published: false
---

# 解説対象

本記事では、トピックを一意に識別する為のトピック名について解説します。

# 前提
- ROS2 humble時の実装に基づいています。
- c++側の実装（rclcppの[node.cpp](https://github.com/ros2/rclcpp/blob/rolling/rclcpp/src/rclcpp/node.cpp)）に基づいていますが、rclpy側も結局はrclで規定されるnode実装につながりますので、大部分は共通です。

# 前提知識

# 公式ドキュメント

- TBD

# ソースの確認


[node_impl.hpp](https://github.com/ros2/rclcpp/blob/humble/rclcpp/include/rclcpp/node_impl.hpp)

```cpp
RCLCPP_LOCAL
inline
std::string
extend_name_with_sub_namespace(const std::string & name, const std::string & sub_namespace)
{
  std::string name_with_sub_namespace(name);
  if (sub_namespace != "" && name.front() != '/' && name.front() != '~') {
    name_with_sub_namespace = sub_namespace + "/" + name;
  }
  return name_with_sub_namespace;
}
```

トピック名から完全修飾トピック名の生成、サービス名から完全修飾サービス名の生成は共に`rcl_expand_topic_name()`という関数で実装されている。(関数名はtopic_nameとなっているがservice名も同じ処理になる)

この関数では下記の順で完全修飾名が生成される

1. プライベート名前空間置換（private namespace substitution）
    - `~`で始まっている場合、`~`がノード名前空間+`/`+ノード名に置換される
2. 予約語置換（substitution） 
    - `{node}`部分はノード名に置換される
    - `{ns}`もしくは`{namesaace}`部分はノード名前空間に置換される
    - ※その他`{}`で囲まれた部分をキー名として辞書で置換する実装があるが、現在の実装では辞書が空。将来予約語が増えることを想定しての実装の様子。
3. 絶対パス化
    - `/`で始まっていない場合、先頭にノード名前空間が足される


[expand_topic_name.c](https://github.com/ros2/rcl/blob/humble/rcl/src/rcl/expand_topic_name.c)

```cpp:expand_topic_name.c
rcl_ret_t
rcl_expand_topic_name(
  const char * input_topic_name,
  const char * node_name,
  const char * node_namespace,
  const rcutils_string_map_t * substitutions,
  rcl_allocator_t allocator,
  char ** output_topic_name)
{
  // 略

  // check if the topic has substitutions to be made
  bool has_a_substitution = strchr(input_topic_name, '{') != NULL;
  bool has_a_namespace_tilde = input_topic_name[0] == '~';
  bool is_absolute = input_topic_name[0] == '/';
  // if absolute and doesn't have any substitution
  if (is_absolute && !has_a_substitution) {
    // nothing to do, duplicate and return
    *output_topic_name = rcutils_strdup(input_topic_name, allocator);
    if (!*output_topic_name) {
      *output_topic_name = NULL;
      RCL_SET_ERROR_MSG("failed to allocate memory for output topic");
      return RCL_RET_BAD_ALLOC;
    }
    return RCL_RET_OK;
  }
  char * local_output = NULL;
  // if has_a_namespace_tilde, replace that first
  if (has_a_namespace_tilde) {
    // special case where node_namespace is just '/'
    // then no additional separating '/' is needed
    const char * fmt = (strlen(node_namespace) == 1) ? "%s%s%s" : "%s/%s%s";
    local_output =
      rcutils_format_string(allocator, fmt, node_namespace, node_name, input_topic_name + 1);
    if (!local_output) {
      *output_topic_name = NULL;
      RCL_SET_ERROR_MSG("failed to allocate memory for output topic");
      return RCL_RET_BAD_ALLOC;
    }
  }
  // if it has any substitutions, replace those
  if (has_a_substitution) {
    // Assumptions entering this scope about the topic string:
    //
    // - All {} are matched and balanced
    // - There is no nesting, i.e. {{}}
    // - There are no empty substitution substr, i.e. '{}' versus '{something}'
    //
    // These assumptions are taken because this is checked in the validation function.
    const char * current_output = (local_output) ? local_output : input_topic_name;
    char * next_opening_brace = NULL;
    // current_output may be replaced on each loop if a substitution is made
    while ((next_opening_brace = strchr(current_output, '{')) != NULL) {
      char * next_closing_brace = strchr(current_output, '}');
      // conclusion based on above assumptions: next_closing_brace - next_opening_brace > 1
      size_t substitution_substr_len = next_closing_brace - next_opening_brace + 1;
      // figure out what the replacement is for this substitution
      const char * replacement = NULL;
      if (strncmp(SUBSTITUION_NODE_NAME, next_opening_brace, substitution_substr_len) == 0) {
        replacement = node_name;
      } else if (  // NOLINT
        strncmp(SUBSTITUION_NAMESPACE, next_opening_brace, substitution_substr_len) == 0 ||
        strncmp(SUBSTITUION_NAMESPACE2, next_opening_brace, substitution_substr_len) == 0)
      {
        replacement = node_namespace;
      } else {
        replacement = rcutils_string_map_getn(
          substitutions,
          // compare {substitution}
          //          ^ until    ^
          next_opening_brace + 1, substitution_substr_len - 2);
        if (!replacement) {
          // in this case, it is neither node name nor ns nor in the substitutions map, so error
          *output_topic_name = NULL;
          char * unmatched_substitution =
            rcutils_strndup(next_opening_brace, substitution_substr_len, allocator);
          if (unmatched_substitution) {
            RCL_SET_ERROR_MSG_WITH_FORMAT_STRING(
              "unknown substitution: %s", unmatched_substitution);
          } else {
            RCUTILS_SAFE_FWRITE_TO_STDERR("failed to allocate memory for unmatched substitution\n");
          }
          allocator.deallocate(unmatched_substitution, allocator.state);
          allocator.deallocate(local_output, allocator.state);
          return RCL_RET_UNKNOWN_SUBSTITUTION;
        }
      }
      // at this point replacement will be set or an error would have returned out
      // do the replacement
      char * next_substitution =
        rcutils_strndup(next_opening_brace, substitution_substr_len, allocator);
      if (!next_substitution) {
        *output_topic_name = NULL;
        RCL_SET_ERROR_MSG("failed to allocate memory for substitution");
        allocator.deallocate(local_output, allocator.state);
        return RCL_RET_BAD_ALLOC;
      }
      char * original_local_output = local_output;
      local_output = rcutils_repl_str(current_output, next_substitution, replacement, &allocator);
      allocator.deallocate(next_substitution, allocator.state);  // free no matter what
      allocator.deallocate(original_local_output, allocator.state);  // free no matter what
      if (!local_output) {
        *output_topic_name = NULL;
        RCL_SET_ERROR_MSG("failed to allocate memory for expanded topic");
        return RCL_RET_BAD_ALLOC;
      }
      current_output = local_output;
      // loop until all substitutions are replaced
    }  // while
  }
  // finally make the name absolute if it isn't already
  if (
    (local_output && local_output[0] != '/') ||
    (!local_output && input_topic_name[0] != '/'))
  {
    char * original_local_output = local_output;
    // special case where node_namespace is just '/'
    // then no additional separating '/' is needed
    const char * fmt = (strlen(node_namespace) == 1) ? "%s%s" : "%s/%s";
    local_output = rcutils_format_string(
      allocator, fmt, node_namespace, (local_output) ? local_output : input_topic_name);
    if (original_local_output) {
      allocator.deallocate(original_local_output, allocator.state);
    }
    if (!local_output) {
      *output_topic_name = NULL;
      RCL_SET_ERROR_MSG("failed to allocate memory for output topic");
      return RCL_RET_BAD_ALLOC;
    }
  }
  // finally store the result in the out pointer and return
  *output_topic_name = local_output;
  return RCL_RET_OK;
}

```


# まとめ

