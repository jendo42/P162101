#include "Arduino.h"
#include "game.h"
#include "P162101.h"
#include "utility.h"

void clearScreen(uint8_t pattern);
void xorPixel(int x, int y);
bool getPixel(int x, int y);
void showMessage(const char *text, uint32_t frames);

typedef struct {
  bool active;
  int x, y;
} bullet;

typedef struct {
  bool active;
  int x, y, t;
} enemy;

static int player_y = 3;
static bullet bullets[16];
static int last_bullet;
static enemy enemies[16];
static int last_enemy;
static int score;

void restart_game(uint64_t frame_cnt)
{
  score = 0;
  player_y = 3;
  seedRand(frame_cnt);
  for (int i = 0; i < countof(bullets); i++) {
    bullets[i].active = false;
  }
  for (int i = 0; i < countof(enemies); i++) {
    enemies[i].active = false;
  }
}

bullet *create_bullet(int y)
{
  int base = last_bullet;
  for (int i = 0; i < countof(bullets); i++) {
    int j = (base + i) % countof(bullets);
    bullet *b = bullets + j;
    if (!b->active) {
      b->y = y;
      b->x = 26;
      b->active = true;
      last_bullet = j;
      return b;
    }
  }
  return NULL;
}

enemy *create_enemy(int y)
{
  int base = last_enemy;
  for (int i = 0; i < countof(enemies); i++) {
    int j = (base + i) % countof(enemies);
    enemy *b = enemies + j;
    if (!b->active) {
      b->y = y;
      b->x = 0;
      b->active = true;
      last_enemy = j;
      return b;
    }
  }
  return NULL;
}

void handle_button_game(int button)
{
  switch(button) {
    case GPIO_B1_BIT:
      if (--player_y < 1) {
        player_y = 1;
      }
      break;
    case GPIO_B3_BIT:
      if (++player_y >= 5) {
        player_y = 5;
      }
      break;
    case GPIO_B2_BIT:
      // fire
      create_bullet(player_y);
      break;
  }
}

bool drawGame(int frame_cnt)
{
  clearScreen(0);

  // draw player
  xorPixel(28, player_y);
  xorPixel(27, player_y);
  xorPixel(26, player_y);
  xorPixel(28, player_y+1);
  xorPixel(28, player_y-1);

  int spawn_coef = score >> 3;
  int speed_coef = spawn_coef;

  if ((frame_cnt & (0x7F >> spawn_coef)) == 0) {
    create_enemy(getRand() % 7);
  }

  // draw bullet
  for (int i = 0; i < countof(bullets); i++) {
    bullet *b = bullets + i;
    if (b->active) {
      xorPixel(b->x, b->y);
      for (int j = 0; j < countof(enemies); j++) {
        enemy *e = enemies + j;
        if (e->active && e->x == b->x && e->y == b->y) {
          e->active = false;
          b->active = false;
          ++score;
        }
      }
      if (--b->x < 0) {
        b->active = false;
      }
    }
  }

  // draw enemy
  for (int i = 0; i < countof(enemies); i++) {
    enemy *e = enemies + i;
    if (e->active) {
      if (e->x >= 26 && getPixel(e->x, e->y)) {
        restart_game(frame_cnt);
        showMessage("GAME OVER", 128 * 4);
        return false;
      }
      xorPixel(e->x, e->y);
      if ((frame_cnt & (0x1F >> speed_coef)) == 0) {
        if (++e->x >= 29) {
          e->active = false;
        }
      }
    }
  }

  return true;
}
