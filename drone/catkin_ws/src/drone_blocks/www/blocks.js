/*
 * Copyright (C) 2020 Copter Express Technologies
 *
 * Author: Oleg Kalachev <okalachev@gmail.com>
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */

const COLOR_FLIGHT = 293;
const COLOR_STATE = 36;
const COLOR_LED = 143;
const COLOR_GPIO = 200;
const DOCS_URL = '';

var frameIds = [["body", "BODY"], ["last navigate target", "NAVIGATE_TARGET"], ["map", "MAP"]];
var frameIdsWithTerrain = frameIds.concat([["terrain", "TERRAIN"]]);

function considerFrameId(e) {
	if (!(e instanceof Blockly.Events.Change || e instanceof Blockly.Events.Create)) return;

	var frameId = this.getFieldValue('FRAME_ID');
	// set appropriate coordinates labels
	if (this.getInput('X')) { // block has x-y-z fields
		if (frameId == 'BODY' || frameId == 'NAVIGATE_TARGET' || frameId == 'BASE_LINK' || frameId == 'TERRAIN') {
			this.getInput('X').fieldRow[0].setValue('вперёд');
			this.getInput('Y').fieldRow[0].setValue('влево');
			this.getInput('Z').fieldRow[0].setValue('вверх');
		} else {
			this.getInput('X').fieldRow[0].setValue('x');
			this.getInput('Y').fieldRow[0].setValue('y');
			this.getInput('Z').fieldRow[0].setValue('z');
		}
		if (this.getInput('LAT')) { // block has global coordinates
			let global = frameId.startsWith('GLOBAL');
			this.getInput('LAT').setVisible(global);
			this.getInput('LON').setVisible(global);
			this.getInput('X').setVisible(!global);
			this.getInput('Y').setVisible(!global);
			this.getInput('Z').fieldRow[0].setValue(frameId == 'GLOBAL' ? 'высота' : 'z');
		}
	}

	this.render();
}

function updateSetpointBlock(e) {
	if (!(e instanceof Blockly.Events.Change || e instanceof Blockly.Events.Create)) return;

	considerFrameId.apply(this, arguments);

	var type = this.getFieldValue('TYPE');
	var velocity = type == 'VELOCITY';
	var attitude = type == 'ATTITUDE' || type == 'RATES';

	this.getInput('VX').setVisible(velocity);
	this.getInput('VY').setVisible(velocity);
	this.getInput('VZ').setVisible(velocity);
	this.getInput('YAW').setVisible(attitude);
	this.getInput('ROLL').setVisible(attitude);
	this.getInput('PITCH').setVisible(attitude);
	this.getInput('THRUST').setVisible(attitude);
	this.getInput('RELATIVE_TO').setVisible(type != 'RATES');

	if (type == 'RATES') {
		this.getInput('ID').setVisible(false);
	}

	this.render();
}

Blockly.Blocks['navigate'] = {
	init: function () {
		let navFrameId = frameIdsWithTerrain.slice();
		navFrameId.push(['global', 'GLOBAL_LOCAL'])
		navFrameId.push(['global, WGS 84 alt.', 'GLOBAL'])
		this.appendDummyInput()
			.appendField("Полёт в точку");
		this.appendValueInput("X")
			.setCheck("Number")
			.appendField("вперёд");
		this.appendValueInput("Y")
			.setCheck("Number")
			.appendField("влево");
		this.appendValueInput("LAT")
			.setCheck("Number")
			.appendField("широта")
			.setVisible(false);
		this.appendValueInput("LON")
			.setCheck("Number")
			.appendField("долгота")
			.setVisible(false)
		this.appendValueInput("Z")
			.setCheck("Number")
			.appendField("вверх");
		this.appendDummyInput()
			.appendField("относительно")
			.appendField(new Blockly.FieldDropdown(navFrameId), "FRAME_ID");
		this.appendValueInput("ID")
			.setCheck("Number")
			.appendField("with ID")
			.setVisible(false)
		this.appendValueInput("SPEED")
			.setCheck("Number")
			.appendField("with speed");
		this.appendDummyInput()
			.appendField("wait")
			.appendField(new Blockly.FieldCheckbox("TRUE"), "WAIT");
		this.setInputsInline(false);
		this.setPreviousStatement(true, null);
		this.setNextStatement(true, null);
		this.setColour(COLOR_FLIGHT);
		this.setTooltip("Полёт в указанную точку, координаты указываются в метрах.");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
		this.setOnChange(considerFrameId);
	}
};

Blockly.Blocks['set_velocity'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("Установить скорость");
		this.appendValueInput("X")
			.setCheck("Number")
			.appendField("forward");
		this.appendValueInput("Y")
			.setCheck("Number")
			.appendField("left");
		this.appendValueInput("Z")
			.setCheck("Number")
			.appendField("up");
		this.appendDummyInput()
			.appendField("относительно")
			.appendField(new Blockly.FieldDropdown(frameIds), "FRAME_ID");
		this.appendValueInput("ID")
			.setCheck("Number")
			.appendField("with ID")
			.setVisible(false)
		this.setInputsInline(false);
		this.setPreviousStatement(true, null);
		this.setNextStatement(true, null);
		this.setColour(COLOR_FLIGHT);
		this.setTooltip("Устанавливает скорость дрона в метрах в секунду (отменяет запросы на полёт).");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
		this.setOnChange(considerFrameId);
	}
};

Blockly.Blocks['setpoint'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("set");
		this.appendDummyInput()
			.appendField(new Blockly.FieldDropdown([["скорость", "VELOCITY"], ["ориентацию", "ATTITUDE"], ["угловые скорости", "RATES"]]), "TYPE");
		this.appendValueInput("VX")
			.setCheck("Number")
			.appendField("vx");
		this.appendValueInput("VY")
			.setCheck("Number")
			.appendField("vy");
		this.appendValueInput("VZ")
			.setCheck("Number")
			.appendField("vz");
		this.appendValueInput("ROLL")
			.setCheck("Number")
			.appendField("крен")
			.setVisible(false);
		this.appendValueInput("PITCH")
			.setCheck("Number")
			.appendField("тангаж")
			.setVisible(false);
		this.appendValueInput("YAW")
			.setCheck("Number")
			.appendField("рыскание")
			.setVisible(false);
		this.appendValueInput("THRUST")
			.setCheck("Number")
			.appendField("газ")
			.setVisible(false);
		this.appendDummyInput('RELATIVE_TO')
			.appendField("относительно")
			.appendField(new Blockly.FieldDropdown(frameIds), "FRAME_ID");
		this.appendValueInput("ID")
			.setCheck("Number")
			.appendField("with ID")
			.setVisible(false);
		this.setInputsInline(false);
		this.setPreviousStatement(true, null);
		this.setNextStatement(true, null);
		this.setColour(COLOR_FLIGHT);
		this.setTooltip("Устанавливает целевые параметры различных типов (отменяет запросы на полёт).");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
		this.setOnChange(updateSetpointBlock);
	}
};

Blockly.Blocks['rangefinder_distance'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("Текущее расстояние до дальномера");
		this.setOutput(true, "Number");
		this.setColour(COLOR_STATE);
		this.setTooltip("");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};

Blockly.Blocks['get_position'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("Текущее положение")
			.appendField(new Blockly.FieldDropdown([["x", "X"], ["y", "Y"], ["z", "Z"], ["vx", "VX"], ["vy", "VY"], ["vz", "VZ"]]), "FIELD")
			.appendField("относительно")
			.appendField(new Blockly.FieldDropdown(frameIdsWithTerrain), "FRAME_ID");
		this.appendValueInput("ID")
			.setCheck("Number")
			.appendField("с ID")
			.setVisible(false)
		this.setOutput(true, "Number");
		this.setColour(COLOR_STATE);
		this.setTooltip("Возвращает текущее положение (в метрах) или скорость (в м/с).");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
		this.setOnChange(considerFrameId);
	}
};

Blockly.Blocks['get_yaw'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("Текущее рыскание относительно")
			.appendField(new Blockly.FieldDropdown(frameIds), "FRAME_ID");
		this.appendValueInput("ID")
			.setCheck("Number")
			.appendField("с ID")
			.setVisible(false)
		this.setOutput(true, "Number");
		this.setColour(COLOR_STATE);
		this.setTooltip("Возвращает текущее рыскание в градусах.");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
		this.setOnChange(considerFrameId);
	}
};

Blockly.Blocks['get_attitude'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("Текущая ориентация")
			.appendField(new Blockly.FieldDropdown([["крен", "ROLL"], ["тангаж", "PITCH"], ["угловая скорость крена", "ROLL_RATE"], ["угловая скорость тангажа", "PITCH_RATE"], ["угловая скорость рыскания", "YAW_RATE"]]), "FIELD");
		this.setOutput(true, "Number");
		this.setColour(COLOR_STATE);
		this.setTooltip("Возвращает текущую ориентацию (в градусах) или угловые скорости (в градусах в секунду).");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};

Blockly.Blocks['voltage'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("Текущее напряжение")
			.appendField(new Blockly.FieldDropdown([["общее", "VOLTAGE"], ["в ячейках", "CELL_VOLTAGE"]]), "TYPE");
		this.setOutput(true, "Number");
		this.setColour(COLOR_STATE);
		this.setTooltip("Возвращает текущий заряд аккумулятора в вольтах.");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};

Blockly.Blocks['get_rc'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("Канал АУ")
		this.appendValueInput("CHANNEL")
			.setCheck("Number");
		this.setInputsInline(true);
		this.setOutput(true, "Number");
		this.setColour(COLOR_STATE);
		this.setTooltip("Возвращает текущее значение канала аппаратуры управления");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
}

Blockly.Blocks['armed'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("Моторы включены?");
		this.setOutput(true, "Boolean");
		this.setColour(COLOR_STATE);
		this.setTooltip("Возвращает логическое значение, указывающее, включены ли моторы дрона.");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};


Blockly.Blocks['mode'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("Текущий режим полёта");
		this.setOutput(true, "String");
		this.setColour(COLOR_STATE);
		this.setTooltip("Возвращает текущий режим полёта (STABILIZE, GUIDED, и т.д.).");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};


Blockly.Blocks['wait_arrival'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("Ждать прибытия");
		this.setPreviousStatement(true, null);
		this.setNextStatement(true, null);
		this.setColour(COLOR_FLIGHT);
		this.setTooltip("Ждать, пока дрон не прилетит к цели полёта.");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};

Blockly.Blocks['get_time'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("Время");
		this.setOutput(true, "Number");
		this.setColour(COLOR_STATE);
		this.setTooltip("Возвращает текущее время в секундах.");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};

Blockly.Blocks['arrived'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("Прибыл?");
		this.setOutput(true, "Boolean");
		this.setColour(COLOR_STATE);
		this.setTooltip("Возвращает логическое значение, указывающее, прибыл ли дрон к цели полёта.");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};

Blockly.Blocks['set_led'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("Установить цвет светодиода");
		this.appendValueInput("INDEX")
			.setCheck("Number");
		this.appendValueInput("COLOR")
			.setCheck("Colour")
			.appendField("на");
		this.setInputsInline(true);
		this.setPreviousStatement(true, null);
		this.setNextStatement(true, null);
		this.setColour(COLOR_LED);
		this.setTooltip("Устанавливает цвет индивидуального светодиода на указанный.");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};

Blockly.Blocks['set_effect'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("Установить эффект светодиодов на")
			.appendField(new Blockly.FieldDropdown([["заполнение", "FILL"], ["мигание", "BLINK"], ["быстрое мигание", "BLINK_FAST"], ["угасание", "FADE"], ["замену", "WIPE"], ["вспышку", "FLASH"], ["радугу", "RAINBOW"], ["заполнение радугой", "RAINBOW_FILL"]]), "EFFECT");
		this.appendValueInput("COLOR")
			.setCheck("Colour")
			.appendField("с цветом");
		this.setInputsInline(true);
		this.setPreviousStatement(true, null);
		this.setNextStatement(true, null);
		this.setColour(COLOR_LED);
		this.setTooltip("Устанавливает желаемый эффект светодиодов.");
		this.setHelpUrl(DOCS_URL + '#' + this.type);

		this.setOnChange(function(e) {
			if (!(e instanceof Blockly.Events.Change || e instanceof Blockly.Events.Create)) return;

			// Hide color field on some effects
			var effect = this.getFieldValue('EFFECT');
			var hideColor = effect == 'RAINBOW' || effect == 'RAINBOW_FILL';
			this.inputList[1].setVisible(!hideColor);
			this.render();
		});
	}
};

Blockly.Blocks['led_count'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("Количество светодиодов");
		this.setOutput(true, "Number");
		this.setColour(COLOR_LED);
		this.setTooltip("Возвращает количество светодиодов (настраивается в led.launch).");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};

Blockly.Blocks['take_off'] = {
	init: function () {
		this.appendValueInput("ALT")
			.setCheck("Number")
			.appendField("Взлёт на");
		this.appendDummyInput()
			.appendField("подождать")
			.appendField(new Blockly.FieldCheckbox("TRUE"), "WAIT");
		this.setPreviousStatement(true, null);
		this.setNextStatement(true, null);
		this.setColour(COLOR_FLIGHT);
		this.setTooltip("Взлёт на желаемую высоту в метрах.");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};

Blockly.Blocks['land'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("Приземлиться");
		this.appendDummyInput()
			.appendField("подождать")
			.appendField(new Blockly.FieldCheckbox("TRUE"), "WAIT");
		this.setInputsInline(true);
		this.setPreviousStatement(true, null);
		this.setNextStatement(true, null);
		this.setColour(COLOR_FLIGHT);
		this.setTooltip("Приземлить дрон.");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};

Blockly.Blocks['global_position'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("Текущая глобальная позиция")
			.appendField(new Blockly.FieldDropdown([["широта", "LAT"], ["долгота", "LON"], ["высота", "ALT"]]), "FIELD");
		this.setOutput(true, "Number");
		this.setColour(COLOR_STATE);
		this.setTooltip("Возвращает текущую глобальную позицию (широта, долгота, высота над эллипсоидом WGS 84).");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};

Blockly.Blocks['on_take_off'] = {
	init: function () {
		this.appendStatementInput("TAKE_OFF")
			.setCheck(null)
			.appendField("При успешном взлёте");
		this.setColour(230);
		this.setTooltip("");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};

Blockly.Blocks['on_landing'] = {
	init: function () {
		this.appendStatementInput("LAND")
			.setCheck(null)
			.appendField("При успешной посадке");
		this.setColour(230);
		this.setTooltip("");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};

Blockly.Blocks['on_armed'] = {
	init: function () {
		this.appendStatementInput("ARMED")
			.setCheck(null)
			.appendField("При включённых моторах");
		this.setColour(230);
		this.setTooltip("");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};

Blockly.FieldAngle.WRAP = 180;
Blockly.FieldAngle.ROUND = 10;

Blockly.Blocks['angle'] = {
	init: function () {
		this.appendDummyInput()
			.appendField(new Blockly.FieldAngle(90), "ANGLE");
		this.setOutput(true, "Number");
		this.setColour(230);
		this.setTooltip("");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};

Blockly.Blocks['set_yaw'] = {
	init: function () {
		this.appendValueInput("YAW")
			.setCheck("Number")
			.appendField("Повернуть на");
		this.appendDummyInput()
			.appendField("относительно")
			.appendField(new Blockly.FieldDropdown([["body", "body"], ["last navigate target", "navigate_target"]]), "FRAME_ID");
		this.appendDummyInput()
			.appendField("wait")
			.appendField(new Blockly.FieldCheckbox("TRUE"), "WAIT");
		this.setInputsInline(true);
		this.setPreviousStatement(true, null);
		this.setNextStatement(true, null);
		this.setColour(COLOR_FLIGHT);
		this.setTooltip("Поворачивает дрон на заданное количество градусов (не радиан).");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};

Blockly.Blocks['distance'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("Расстояние до точки");
		this.appendValueInput("X")
			.setCheck("Number")
			.appendField("x");
		this.appendValueInput("Y")
			.setCheck("Number")
			.appendField("y");
		this.appendValueInput("Z")
			.setCheck("Number")
			.appendField("z");
		this.appendDummyInput()
			.appendField("относительно")
			.appendField(new Blockly.FieldDropdown([["last navigate target", "NAVIGATE_TARGET"], ["terrain", "TERRAIN"]]), "FRAME_ID");
		this.appendValueInput("ID")
			.setCheck("Number")
			.appendField("с ID")
			.setVisible(false);
		this.setInputsInline(false);
		this.setOutput(true, "Number");
		this.setColour(COLOR_STATE);
		this.setTooltip("Возвращает расстояние до указанной точки в метрах.");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
		this.setOnChange(considerFrameId);
	}
};

Blockly.Blocks['wait'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("Ждать");
		this.appendValueInput("TIME")
			.setCheck("Number");
		this.appendDummyInput()
			.appendField("секунд");
		this.setPreviousStatement(true, null);
		this.setNextStatement(true, null);
		this.setColour(COLOR_FLIGHT);
		this.setTooltip("");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};

var keys = [['up', 'UP'], ['down', 'DOWN'], ['left', 'LEFT'], ['right', 'RIGHT'], ['space', 'SPACE']];

Blockly.Blocks['key_pressed'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("Клавиша")
			.appendField(new Blockly.FieldDropdown(keys, "NAME"))
			.appendField("нажата");
		this.appendStatementInput("PRESSED")
			.setCheck(null);
		this.setPreviousStatement(true, null);
		this.setNextStatement(true, null);
		this.setColour(230);
		this.setTooltip("");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};

Blockly.Blocks['gpio_read'] = {
	init: function () {
		this.appendValueInput("PIN")
			.setCheck("Number")
			.appendField("Прочитать контакт GPIO");
		this.setOutput(true, "Boolean");
		this.setColour(COLOR_GPIO);
		this.setTooltip("Возвращает логическое значение, указывающее, есть ли напряжение на контакте GPIO.");
		this.setHelpUrl(DOCS_URL + '#GPIO');
	}
};

Blockly.Blocks['gpio_write'] = {
	init: function () {
		this.appendValueInput("PIN")
			.setCheck("Number")
			.appendField("Установить контакт GPIO");
		this.appendValueInput("LEVEL")
			.setCheck("Boolean")
			.appendField("to");
		this.setInputsInline(true);
		this.setColour(COLOR_GPIO);
		this.setPreviousStatement(true, null);
		this.setNextStatement(true, null);
		this.setTooltip("Записывает значение в контакт GPIO.");
		this.setHelpUrl(DOCS_URL + '#GPIO');
	}
};

Blockly.Blocks['set_servo'] = {
	init: function () {
		this.appendValueInput("PIN")
			.setCheck("Number")
			.appendField("Установить контакт GPIO");
		this.appendValueInput("PWM")
			.setCheck("Number")
			.appendField("в значение ШИМ");
		this.setInputsInline(true);
		this.setColour(COLOR_GPIO);
		this.setPreviousStatement(true, null);
		this.setNextStatement(true, null);
		this.setTooltip("Записывает значение ШИМ в контакт GPIO для управления сервоприводом. ШИМ указывается в диапазоне 500–2500 мкс.");
		this.setHelpUrl(DOCS_URL + '#GPIO');
	}
};

Blockly.Blocks['set_duty_cycle'] = {
	init: function () {
		this.appendValueInput("PIN")
			.setCheck("Number")
			.appendField("Установить контакт GPIO");
		this.appendValueInput("DUTY_CYCLE")
			.setCheck("Number")
			.appendField("в значение коэффициента заполнения");
		this.setInputsInline(true);
		this.setColour(COLOR_GPIO);
		this.setPreviousStatement(true, null);
		this.setNextStatement(true, null);
		this.setTooltip("Записывает значение коэффициента заполнения ШИМ в контакт GPIO (для управления яркостью светодиодов, и т.д.). Коэффициент заполнения указывается в диапазоне 0.0 - 1.0.");
		this.setHelpUrl(DOCS_URL + '#GPIO');
	}
};
