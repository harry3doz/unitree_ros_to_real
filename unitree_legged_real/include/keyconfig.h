#pragma once

#include <map>

class WpKey
{
public:

  enum Type
  {
    QUIT,
    UP,
    LEFT,
    DOWN,
    RIGHT,
    ROT_L,
    ROT_R,
    WP_ADD,
    WP_DEL,
    WP_CLEAR,
    WP_INIT,
    WP_NEXT,
    WP_PREV,
    WP_SAVE,
    WP_LOAD,
    INIT_SWITCH,
    TYPE_SWITCH,
    NAV_START,
    NAV_STOP,
    DEFAULT
  };

  static const Type wpcfg(char input)
  {

    static const std::map<char, Type> name= 
    {
      {'q', QUIT},
      {'e', UP},
      {'s', LEFT},
      {'d', DOWN},
      {'f', RIGHT},
      {'j', ROT_L},
      {'l', ROT_R},
      {'k', WP_ADD},
      {'o', WP_DEL},
      {'u', WP_CLEAR},
      {'i', WP_INIT},
      {'n', WP_NEXT},
      {'b', WP_PREV},
      {'0', WP_SAVE},
      {'5', WP_LOAD},
      {'r', INIT_SWITCH},
      {'t', TYPE_SWITCH},
      {'7', NAV_START},
      {'9', NAV_STOP},
    };

    if (name.find(input) != name.end()) return name.at(input);
    else return DEFAULT;
  }

  Type state;

};
