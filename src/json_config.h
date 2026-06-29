#ifndef JSON_CONFIG_H
#define JSON_CONFIG_H

#include <ctype.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * Minimal self-contained JSON helper layer compatible with the tiny API
 * used by the Thingino motor tools.
 */

typedef enum {
  JSON_NULL = 0,
  JSON_BOOL,
  JSON_NUMBER,
  JSON_STRING,
  JSON_OBJECT,
  JSON_ARRAY,
} JsonType;

typedef enum {
  JSON_NUMBER_INT = 0,
  JSON_NUMBER_REAL,
} JsonNumberKind;

typedef struct {
  JsonNumberKind kind;
  long integer;
  double real;
} JsonNumber;

typedef struct JsonValue JsonValue;

typedef struct JsonObjectItem {
  char *name;
  JsonValue *value;
  struct JsonObjectItem *next;
} JsonObjectItem;

struct JsonValue {
  JsonType type;
  union {
    bool boolean;
    char *string;
    JsonNumber number;
    JsonObjectItem *object;
    struct {
      JsonValue **items;
      size_t length;
    } array;
  } value;
};

static char *json_config_strndup(const char *src, size_t len) {
  char *out = (char *)malloc(len + 1);
  if (!out)
    return NULL;
  memcpy(out, src, len);
  out[len] = '\0';
  return out;
}

static JsonValue *json_config_new_value(JsonType type) {
  JsonValue *value = (JsonValue *)calloc(1, sizeof(JsonValue));
  if (!value)
    return NULL;
  value->type = type;
  return value;
}

static void free_json_value(JsonValue *value);

static void json_config_free_object(JsonObjectItem *item) {
  while (item) {
    JsonObjectItem *next = item->next;
    free(item->name);
    free_json_value(item->value);
    free(item);
    item = next;
  }
}

static void json_config_free_array(JsonValue **items, size_t length) {
  if (!items)
    return;
  for (size_t i = 0; i < length; ++i)
    free_json_value(items[i]);
  free(items);
}

static void free_json_value(JsonValue *value) {
  if (!value)
    return;

  switch (value->type) {
  case JSON_STRING:
    free(value->value.string);
    break;
  case JSON_OBJECT:
    json_config_free_object(value->value.object);
    break;
  case JSON_ARRAY:
    json_config_free_array(value->value.array.items, value->value.array.length);
    break;
  default:
    break;
  }

  free(value);
}

static void json_config_skip_ws(const char **p) {
  while (*p && **p && isspace((unsigned char)**p))
    (*p)++;
}

static int json_config_hex_value(int c) {
  if (c >= '0' && c <= '9')
    return c - '0';
  if (c >= 'a' && c <= 'f')
    return 10 + (c - 'a');
  if (c >= 'A' && c <= 'F')
    return 10 + (c - 'A');
  return -1;
}

static int json_config_buf_append(char **buf, size_t *len, size_t *cap,
                                  char ch) {
  if (*len + 1 >= *cap) {
    size_t new_cap = (*cap == 0) ? 32 : (*cap * 2);
    char *tmp = (char *)realloc(*buf, new_cap);
    if (!tmp)
      return 0;
    *buf = tmp;
    *cap = new_cap;
  }
  (*buf)[(*len)++] = ch;
  (*buf)[*len] = '\0';
  return 1;
}

static JsonValue *json_config_parse_value(const char **p);

static JsonValue *json_config_parse_string(const char **p) {
  if (!p || !*p || **p != '"')
    return NULL;

  (*p)++;
  char *buf = NULL;
  size_t len = 0;
  size_t cap = 0;

  while (**p && **p != '"') {
    char ch = *(*p)++;
    if (ch == '\\') {
      char esc = *(*p)++;
      if (!esc) {
        free(buf);
        return NULL;
      }
      switch (esc) {
      case '"': ch = '"'; break;
      case '\\': ch = '\\'; break;
      case '/': ch = '/'; break;
      case 'b': ch = '\b'; break;
      case 'f': ch = '\f'; break;
      case 'n': ch = '\n'; break;
      case 'r': ch = '\r'; break;
      case 't': ch = '\t'; break;
      case 'u': {
        int code = 0;
        for (int i = 0; i < 4; ++i) {
          int hex = json_config_hex_value(*(*p)++);
          if (hex < 0) {
            free(buf);
            return NULL;
          }
          code = (code << 4) | hex;
        }
        if (code < 0x80) {
          ch = (char)code;
          if (!json_config_buf_append(&buf, &len, &cap, ch)) {
            free(buf);
            return NULL;
          }
          continue;
        }
        ch = '?';
        break;
      }
      default:
        free(buf);
        return NULL;
      }
    }

    if (!json_config_buf_append(&buf, &len, &cap, ch)) {
      free(buf);
      return NULL;
    }
  }

  if (**p != '"') {
    free(buf);
    return NULL;
  }
  (*p)++;

  JsonValue *value = json_config_new_value(JSON_STRING);
  if (!value) {
    free(buf);
    return NULL;
  }
  value->value.string = buf ? buf : json_config_strndup("", 0);
  if (!value->value.string) {
    free(value);
    return NULL;
  }
  return value;
}

static JsonValue *json_config_parse_number(const char **p) {
  const char *start = *p;
  char *end = NULL;
  errno = 0;
  double real_value = strtod(start, &end);
  if (end == start || errno == ERANGE)
    return NULL;

  JsonValue *value = json_config_new_value(JSON_NUMBER);
  if (!value)
    return NULL;

  bool is_integer = true;
  for (const char *it = start; it < end; ++it) {
    if (*it == '.' || *it == 'e' || *it == 'E') {
      is_integer = false;
      break;
    }
  }

  if (is_integer) {
    value->value.number.kind = JSON_NUMBER_INT;
    value->value.number.integer = strtol(start, NULL, 10);
    value->value.number.real = (double)value->value.number.integer;
  } else {
    value->value.number.kind = JSON_NUMBER_REAL;
    value->value.number.real = real_value;
    value->value.number.integer = (long)real_value;
  }

  *p = end;
  return value;
}

static JsonValue *json_config_parse_literal(const char **p, const char *lit,
                                           JsonType type, bool boolean_value) {
  size_t len = strlen(lit);
  if (strncmp(*p, lit, len) != 0)
    return NULL;
  *p += len;

  JsonValue *value = json_config_new_value(type);
  if (!value)
    return NULL;
  if (type == JSON_BOOL)
    value->value.boolean = boolean_value;
  return value;
}

static JsonValue *json_config_parse_array(const char **p) {
  if (**p != '[')
    return NULL;
  (*p)++;
  json_config_skip_ws(p);

  JsonValue *value = json_config_new_value(JSON_ARRAY);
  if (!value)
    return NULL;

  while (**p && **p != ']') {
    JsonValue *item = json_config_parse_value(p);
    if (!item) {
      free_json_value(value);
      return NULL;
    }
    JsonValue **items = (JsonValue **)realloc(
        value->value.array.items,
        sizeof(JsonValue *) * (value->value.array.length + 1));
    if (!items) {
      free_json_value(item);
      free_json_value(value);
      return NULL;
    }
    value->value.array.items = items;
    value->value.array.items[value->value.array.length++] = item;

    json_config_skip_ws(p);
    if (**p == ',') {
      (*p)++;
      json_config_skip_ws(p);
      continue;
    }
    break;
  }

  if (**p != ']') {
    free_json_value(value);
    return NULL;
  }
  (*p)++;
  return value;
}

static JsonValue *json_config_parse_object(const char **p) {
  if (**p != '{')
    return NULL;
  (*p)++;
  json_config_skip_ws(p);

  JsonValue *value = json_config_new_value(JSON_OBJECT);
  if (!value)
    return NULL;

  JsonObjectItem *tail = NULL;

  while (**p && **p != '}') {
    JsonValue *key = json_config_parse_string(p);
    if (!key) {
      free_json_value(value);
      return NULL;
    }
    char *name = key->value.string;
    key->value.string = NULL;
    free_json_value(key);

    json_config_skip_ws(p);
    if (**p != ':') {
      free(name);
      free_json_value(value);
      return NULL;
    }
    (*p)++;
    json_config_skip_ws(p);

    JsonValue *member = json_config_parse_value(p);
    if (!member) {
      free(name);
      free_json_value(value);
      return NULL;
    }

    JsonObjectItem *entry = (JsonObjectItem *)calloc(1, sizeof(JsonObjectItem));
    if (!entry) {
      free(name);
      free_json_value(member);
      free_json_value(value);
      return NULL;
    }
    entry->name = name;
    entry->value = member;

    if (!value->value.object)
      value->value.object = entry;
    else
      tail->next = entry;
    tail = entry;

    json_config_skip_ws(p);
    if (**p == ',') {
      (*p)++;
      json_config_skip_ws(p);
      continue;
    }
    break;
  }

  if (**p != '}') {
    free_json_value(value);
    return NULL;
  }
  (*p)++;
  return value;
}

static JsonValue *json_config_parse_value(const char **p) {
  json_config_skip_ws(p);
  if (!p || !*p || !**p)
    return NULL;

  switch (**p) {
  case '{':
    return json_config_parse_object(p);
  case '[':
    return json_config_parse_array(p);
  case '"':
    return json_config_parse_string(p);
  case 't':
    return json_config_parse_literal(p, "true", JSON_BOOL, true);
  case 'f':
    return json_config_parse_literal(p, "false", JSON_BOOL, false);
  case 'n':
    return json_config_parse_literal(p, "null", JSON_NULL, false);
  default:
    return json_config_parse_number(p);
  }
}

static JsonValue *parse_json_file(const char *path) {
  FILE *fp = fopen(path, "r");
  if (!fp)
    return NULL;

  if (fseek(fp, 0, SEEK_END) != 0) {
    fclose(fp);
    return NULL;
  }
  long size = ftell(fp);
  if (size < 0) {
    fclose(fp);
    return NULL;
  }
  rewind(fp);

  char *data = (char *)malloc((size_t)size + 1);
  if (!data) {
    fclose(fp);
    return NULL;
  }

  size_t read_bytes = fread(data, 1, (size_t)size, fp);
  fclose(fp);
  data[read_bytes] = '\0';

  const char *cursor = data;
  JsonValue *value = json_config_parse_value(&cursor);
  json_config_skip_ws(&cursor);
  if (!value || *cursor != '\0') {
    free_json_value(value);
    free(data);
    return NULL;
  }

  free(data);
  return value;
}

static JsonValue *get_object_item(JsonValue *obj, const char *key) {
  if (!obj || obj->type != JSON_OBJECT || !key)
    return NULL;

  for (JsonObjectItem *item = obj->value.object; item; item = item->next) {
    if (item->name && strcmp(item->name, key) == 0)
      return item->value;
  }
  return NULL;
}

#endif /* JSON_CONFIG_H */
