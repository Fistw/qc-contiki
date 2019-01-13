
// System configuration
static struct uwbConfig_s config =
{
address:
    {0, 0, 0, 0, 0, 0, 0xcf, 0xbc},
};

struct uwbConfig_s *uwbGetConfig()
{
    config.positionEnabled = 1;
    return &config;
}
