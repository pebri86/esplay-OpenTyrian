#include "SDL_event.h"
#include "sdkconfig.h"
#define NELEMS(x) (sizeof(x) / sizeof((x)[0]))

typedef struct
{
    int gpio;
    SDL_Scancode scancode;
    SDL_Keycode keycode;
} GPIOKeyMap;

int keyMode = 1;
//Mappings from buttons to keys
#ifdef CONFIG_HW_ODROID_GO
static const GPIOKeyMap keymap[2][6] = {{
                                            // Game
                                            {CONFIG_HW_BUTTON_PIN_NUM_BUTTON1, SDL_SCANCODE_LCTRL, SDLK_LCTRL},
                                            {CONFIG_HW_BUTTON_PIN_NUM_SELECT, SDL_SCANCODE_SPACE, SDLK_SPACE},
                                            {CONFIG_HW_BUTTON_PIN_NUM_VOL, SDL_SCANCODE_CAPSLOCK, SDLK_CAPSLOCK},
                                            {CONFIG_HW_BUTTON_PIN_NUM_MENU, SDL_SCANCODE_ESCAPE, SDLK_ESCAPE},
                                            {CONFIG_HW_BUTTON_PIN_NUM_START, SDL_SCANCODE_A, SDLK_a},
                                            {CONFIG_HW_BUTTON_PIN_NUM_BUTTON2, SDL_SCANCODE_LALT, SDLK_LALT},
                                        },
                                        // Menu
                                        {
                                            {CONFIG_HW_BUTTON_PIN_NUM_BUTTON1, SDL_SCANCODE_SPACE, SDLK_SPACE},
                                            {CONFIG_HW_BUTTON_PIN_NUM_BUTTON2, SDL_SCANCODE_ESCAPE, SDLK_ESCAPE},
                                            {CONFIG_HW_BUTTON_PIN_NUM_VOL, SDL_SCANCODE_ESCAPE, SDLK_ESCAPE},
                                            {CONFIG_HW_BUTTON_PIN_NUM_MENU, SDL_SCANCODE_ESCAPE, SDLK_ESCAPE},
                                            {CONFIG_HW_BUTTON_PIN_NUM_START, SDL_SCANCODE_A, SDLK_a},
                                            {CONFIG_HW_BUTTON_PIN_NUM_SELECT, SDL_SCANCODE_LALT, SDLK_LALT},
                                        }};
#else
#ifdef CONFIG_HW_ESPLAY_MICRO
static const GPIOKeyMap keymap[2][3] = {{
                                            // Game
                                            {GPIO_NUM_34, SDL_SCANCODE_R, SDLK_r},           // R
                                            {GPIO_NUM_35, SDL_SCANCODE_ESCAPE, SDLK_ESCAPE}, // Menu
                                            {GPIO_NUM_36, SDL_SCANCODE_L, SDLK_l},           // L
                                        },
                                        // Menu
                                        {
                                            {GPIO_NUM_34, SDL_SCANCODE_R, SDLK_r},           // R
                                            {GPIO_NUM_35, SDL_SCANCODE_ESCAPE, SDLK_ESCAPE}, // Menu
                                            {GPIO_NUM_36, SDL_SCANCODE_L, SDLK_l},           // L
                                        }};
#else
static const GPIOKeyMap keymap[2][6] = {{
                                            // Game
                                            {CONFIG_HW_BUTTON_PIN_NUM_UP, SDL_SCANCODE_UP, SDLK_UP},
                                            {CONFIG_HW_BUTTON_PIN_NUM_RIGHT, SDL_SCANCODE_RIGHT, SDLK_RIGHT},
                                            {CONFIG_HW_BUTTON_PIN_NUM_DOWN, SDL_SCANCODE_DOWN, SDLK_DOWN},
                                            {CONFIG_HW_BUTTON_PIN_NUM_LEFT, SDL_SCANCODE_LEFT, SDLK_LEFT},

                                            {CONFIG_HW_BUTTON_PIN_NUM_BUTTON1, SDL_SCANCODE_LCTRL, SDLK_LCTRL},
                                            {CONFIG_HW_BUTTON_PIN_NUM_BUTTON2, SDL_SCANCODE_SPACE, SDLK_SPACE},
                                        },
                                        // Menu
                                        {
                                            {CONFIG_HW_BUTTON_PIN_NUM_UP, SDL_SCANCODE_UP, SDLK_UP},
                                            {CONFIG_HW_BUTTON_PIN_NUM_RIGHT, SDL_SCANCODE_RIGHT, SDLK_RIGHT},
                                            {CONFIG_HW_BUTTON_PIN_NUM_DOWN, SDL_SCANCODE_DOWN, SDLK_DOWN},
                                            {CONFIG_HW_BUTTON_PIN_NUM_LEFT, SDL_SCANCODE_LEFT, SDLK_LEFT},

                                            {CONFIG_HW_BUTTON_PIN_NUM_BUTTON2, SDL_SCANCODE_ESCAPE, SDLK_ESCAPE},
                                            {CONFIG_HW_BUTTON_PIN_NUM_BUTTON1, SDL_SCANCODE_SPACE, SDLK_SPACE},
                                        }};
#endif
#endif

#define ODROID_GAMEPAD_IO_X ADC1_CHANNEL_6
#define ODROID_GAMEPAD_IO_Y ADC1_CHANNEL_7

#ifdef CONFIG_HW_ESPLAY_MICRO
enum
{
    GAMEPAD_INPUT_START = 0,
    GAMEPAD_INPUT_SELECT,
    GAMEPAD_INPUT_UP,
    GAMEPAD_INPUT_DOWN,
    GAMEPAD_INPUT_LEFT,
    GAMEPAD_INPUT_RIGHT,
    GAMEPAD_INPUT_A,
    GAMEPAD_INPUT_B,

    GAMEPAD_INPUT_MAX
};

typedef struct
{
    uint8_t i2c_key[GAMEPAD_INPUT_MAX];
    uint8_t buttons[3];
} JoystickState;

JoystickState lastState = {{0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0}};

#else
typedef struct
{
    uint8_t up;
    uint8_t right;
    uint8_t down;
    uint8_t left;
    uint8_t buttons[6];
} JoystickState;

JoystickState lastState = {0, 0, 0, 0, {0, 0, 0, 0, 0, 0}};

#endif

typedef struct
{
    Uint32 type; /**< ::SDL_KEYDOWN or ::SDL_KEYUP */
    SDL_Scancode scancode;
    SDL_Scancode keycode;
} GPIOEvent;

bool initInput = false;

static xQueueHandle gpio_evt_queue = NULL;

int checkPin(int state, uint8_t *lastState, SDL_Scancode sc, SDL_Keycode kc, SDL_Event *event)
{
    if (state != *lastState)
    {
        *lastState = state;
        event->key.keysym.scancode = sc;
        event->key.keysym.sym = kc;
        event->key.type = state ? SDL_KEYDOWN : SDL_KEYUP;
        event->key.state = state ? SDL_PRESSED : SDL_RELEASED;
        return 1;
    }
    return 0;
}

int checkPinStruct(int i, uint8_t *lastState, SDL_Event *event)
{
    int state = 1 - gpio_get_level(keymap[keyMode][i].gpio);
    if (state != *lastState)
    {
        *lastState = state;
        event->key.keysym.scancode = keymap[keyMode][i].scancode;
        event->key.keysym.sym = keymap[keyMode][i].keycode;
        event->key.type = state ? SDL_KEYDOWN : SDL_KEYUP;
        event->key.state = state ? SDL_PRESSED : SDL_RELEASED;
        return 1;
    }
    return 0;
}

#ifdef CONFIG_HW_ODROID_GO
int readOdroidXY(SDL_Event *event)
{
    int joyX = adc1_get_raw(ODROID_GAMEPAD_IO_X);
    int joyY = adc1_get_raw(ODROID_GAMEPAD_IO_Y);

    JoystickState state;
    if (joyX > 2048 + 1024)
    {
        state.left = 1;
        state.right = 0;
    }
    else if (joyX > 1024)
    {
        state.left = 0;
        state.right = 1;
    }
    else
    {
        state.left = 0;
        state.right = 0;
    }

    if (joyY > 2048 + 1024)
    {
        state.up = 1;
        state.down = 0;
    }
    else if (joyY > 1024)
    {
        state.up = 0;
        state.down = 1;
    }
    else
    {
        state.up = 0;
        state.down = 0;
    }

    event->key.keysym.mod = 0;
    if (checkPin(state.up, &lastState.up, SDL_SCANCODE_UP, SDLK_UP, event))
        return 1;
    if (checkPin(state.down, &lastState.down, SDL_SCANCODE_DOWN, SDLK_DOWN, event))
        return 1;
    if (checkPin(state.left, &lastState.left, SDL_SCANCODE_LEFT, SDLK_LEFT, event))
        return 1;
    if (checkPin(state.right, &lastState.right, SDL_SCANCODE_RIGHT, SDLK_RIGHT, event))
        return 1;

    for (int i = 0; i < 6; i++)
        if (checkPinStruct(i, &lastState.buttons[i], event))
            return 1;

    return 0;
}
#endif

#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define WRITE_BIT I2C_MASTER_WRITE  /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ    /*!< I2C master read */
#define ACK_CHECK_EN 0x1            /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0           /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                 /*!< I2C ack value */
#define NACK_VAL 0x1                /*!< I2C nack value */

static gpio_num_t i2c_gpio_sda = 21;
static gpio_num_t i2c_gpio_scl = 22;
static uint32_t i2c_frequency = 100000;
static i2c_port_t i2c_port = I2C_NUM_0;

static esp_err_t i2c_master_driver_initialize()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = i2c_gpio_sda,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = i2c_gpio_scl,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = i2c_frequency};
    return i2c_param_config(i2c_port, &conf);
}

static uint8_t i2c_keypad_read()
{
    int len = 1;
    uint8_t *data = malloc(len);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, 0x20 << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data + len - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    uint8_t val = data[0];
    free(data);

    return val;
}

int readI2CKeypad(SDL_Event *event)
{
    JoystickState state;

    uint8_t i2c_data = i2c_keypad_read();
    for (int i = 0; i < GAMEPAD_INPUT_MAX; ++i)
    {
        if (((1 << i) & i2c_data) == 0)
            state.i2c_key[i] = 1;
        else
            state.i2c_key[i] = 0;
    }

    event->key.keysym.mod = 0;
    if (checkPin(state.i2c_key[GAMEPAD_INPUT_UP], &lastState.i2c_key[GAMEPAD_INPUT_UP], SDL_SCANCODE_UP, SDLK_UP, event))
        return 1;
    if (checkPin(state.i2c_key[GAMEPAD_INPUT_DOWN], &lastState.i2c_key[GAMEPAD_INPUT_DOWN], SDL_SCANCODE_DOWN, SDLK_DOWN, event))
        return 1;
    if (checkPin(state.i2c_key[GAMEPAD_INPUT_LEFT], &lastState.i2c_key[GAMEPAD_INPUT_LEFT], SDL_SCANCODE_LEFT, SDLK_LEFT, event))
        return 1;
    if (checkPin(state.i2c_key[GAMEPAD_INPUT_RIGHT], &lastState.i2c_key[GAMEPAD_INPUT_RIGHT], SDL_SCANCODE_RIGHT, SDLK_RIGHT, event))
        return 1;
    if (keyMode)
    {
        if (checkPin(state.i2c_key[GAMEPAD_INPUT_START], &lastState.i2c_key[GAMEPAD_INPUT_START], SDL_SCANCODE_Y, SDLK_y, event))
            return 1;
        if (checkPin(state.i2c_key[GAMEPAD_INPUT_SELECT], &lastState.i2c_key[GAMEPAD_INPUT_SELECT], SDL_SCANCODE_N, SDLK_n, event))
            return 1;
        if (checkPin(state.i2c_key[GAMEPAD_INPUT_A], &lastState.i2c_key[GAMEPAD_INPUT_A], SDL_SCANCODE_SPACE, SDLK_SPACE, event))
            return 1;
        if (checkPin(state.i2c_key[GAMEPAD_INPUT_B], &lastState.i2c_key[GAMEPAD_INPUT_B], SDL_SCANCODE_DELETE, SDLK_DELETE, event))
            return 1;
    }
    else
    {
        if (checkPin(state.i2c_key[GAMEPAD_INPUT_START], &lastState.i2c_key[GAMEPAD_INPUT_START], SDL_SCANCODE_A, SDLK_a, event))
            return 1;
        if (checkPin(state.i2c_key[GAMEPAD_INPUT_SELECT], &lastState.i2c_key[GAMEPAD_INPUT_SELECT], SDL_SCANCODE_SPACE, SDLK_SPACE, event))
            return 1;
        if (checkPin(state.i2c_key[GAMEPAD_INPUT_A], &lastState.i2c_key[GAMEPAD_INPUT_A], SDL_SCANCODE_LCTRL, SDLK_LCTRL, event))
            return 1;
        if (checkPin(state.i2c_key[GAMEPAD_INPUT_B], &lastState.i2c_key[GAMEPAD_INPUT_B], SDL_SCANCODE_LALT, SDLK_LALT, event))
            return 1;
    }

    for (int i = 0; i < 3; i++)
        if (checkPinStruct(i, &lastState.buttons[i], event))
            return 1;

    return 0;
}

int SDL_PollEvent(SDL_Event *event)
{
    if (!initInput)
        inputInit();

#ifndef CONFIG_HW_ODROID_GO
#ifdef CONFIG_HW_ESPLAY_MICRO
    return readI2CKeypad(event);
#else
    GPIOEvent ev;
    if (xQueueReceive(gpio_evt_queue, &ev, 0))
    {
        event->key.keysym.sym = ev.keycode;
        event->key.keysym.scancode = ev.scancode;
        event->key.type = ev.type;
        event->key.keysym.mod = 0;
        event->key.state = ev.type == SDL_KEYDOWN ? SDL_PRESSED : SDL_RELEASED; //< ::SDL_PRESSED or ::SDL_RELEASED
        return 1;
    }
#endif
#else
    return readOdroidXY(event);
#endif
    return 0;
}

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    GPIOEvent event;
    event.type = gpio_get_level(gpio_num) == 0 ? SDL_KEYDOWN : SDL_KEYUP;
    for (int i = 0; i < NELEMS(keymap[keyMode]); i++)
        if (keymap[keyMode][i].gpio == gpio_num)
        {
            event.scancode = keymap[keyMode][i].scancode;
            event.keycode = keymap[keyMode][i].keycode;
            xQueueSendFromISR(gpio_evt_queue, &event, NULL);
        }
}
/*
void gpioTask(void *arg) {
    uint32_t io_num;
	int level;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
        }
    }
}
*/
void inputInit()
{
    gpio_config_t io_conf;
    io_conf.pull_down_en = 0;

    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_ANYEDGE;

    //bit mask of the pins, use GPIO... here
    for (int i = 0; i < NELEMS(keymap[0]); i++)
        if (i == 0)
            io_conf.pin_bit_mask = (1ULL << keymap[0][i].gpio);
        else
            io_conf.pin_bit_mask |= (1ULL << keymap[0][i].gpio);

    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

#ifndef CONFIG_HW_ODROID_GO
#ifdef CONFIG_HW_ESPLAY_MICRO
    i2c_master_driver_initialize();
    i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
#else
    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(GPIOEvent));
    //start gpio task
    //xTaskCreatePinnedToCore(&gpioTask, "GPIO", 1500, NULL, 7, NULL, 0);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_SHARED);

    //hook isr handler
    for (int i = 0; i < NELEMS(keymap[0]); i++)
        gpio_isr_handler_add(keymap[0][i].gpio, gpio_isr_handler, (void *)keymap[0][i].gpio);
#endif
#else
    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(ODROID_GAMEPAD_IO_X, ADC_ATTEN_11db);
    adc1_config_channel_atten(ODROID_GAMEPAD_IO_Y, ADC_ATTEN_11db);
#endif

    printf("keyboard: GPIO task created.\n");
    initInput = true;
}