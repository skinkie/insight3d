const int UI_ICON_WIDTH = 16;
const int UI_ICON_HEIGHT = 16;
enum UI_ICONS_QUALITY { UI_ICONS_NEUTRAL = 0,
    UI_ICONS_GOODGOOD = 1,
    UI_ICONS_GOOD = 2,
    UI_ICONS_BAD = 3,
    UI_ICONS_BADBAD = 4 };
enum UI_ICONS_BOOL { UI_ICONS_TRUE = 0,
    UI_ICONS_FALSE = 5 };
const int UI_ICONS_QUALITY_COUNT = 5;
const int UI_ICONS_COUNT = 10;

bool ui_icons_initialize();
