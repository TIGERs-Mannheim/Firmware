#include "variant.h"
#include "struct_ids.h"

static void registerShellCommands(ShellCmdHandler* pHandler);

Variant variant = {
	.config = {
		.hasPlanetaryGear = 0,
	},
};

static const ConfigFileDesc configFileDescVariant =
	{ SID_CFG_VARIANT_V2020, 0, "variant", 1, (ElementDesc[]) {
		{ UINT8,  "has_planetary_gear", "bool", "has_planetary_gear" },
	} };

void VariantInit()
{
	variant.pConfigFile = ConfigOpenOrCreate(&configFileDescVariant, &variant.config, sizeof(VariantConfig), 0, CONFIG_FILE_FLAG_INTERNAL);

	ShellCmdHandlerInit(&variant.cmdHandler, 0);
	registerShellCommands(&variant.cmdHandler);
}

SHELL_CMD(dd, "Set gear to direct-drive variant");

SHELL_CMD_IMPL(dd)
{
	(void)pUser; (void)argc; (void)argv;

	variant.config.hasPlanetaryGear = 0;
	ConfigNotifyUpdate(variant.pConfigFile);

	printf("Variant set to direct-drive\r\n");

	chThdSleepMilliseconds(100);

	NVIC_SystemReset();
}

SHELL_CMD(pg, "Set gear to planetary-gear variant");

SHELL_CMD_IMPL(pg)
{
	(void)pUser; (void)argc; (void)argv;

	variant.config.hasPlanetaryGear = 1;
	ConfigNotifyUpdate(variant.pConfigFile);

	printf("Variant set to planetary-gear\r\n");

	chThdSleepMilliseconds(100);

	NVIC_SystemReset();
}

SHELL_CMD(show, "Show current variant settings");

SHELL_CMD_IMPL(show)
{
	(void)pUser; (void)argc; (void)argv;

	printf("Drive: %s\r\n", variant.config.hasPlanetaryGear ? "geared" : "direct");
}

static void registerShellCommands(ShellCmdHandler* pHandler)
{
	ShellCmdAdd(pHandler, show_command);
	ShellCmdAdd(pHandler, dd_command);
	ShellCmdAdd(pHandler, pg_command);
}
