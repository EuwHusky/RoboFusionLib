#include "stdlib.h"

#include "dev_base_motor.h"

static void RflBaseMotorSetSpeed(void *base, float set_speed);
static void RflBaseMotorSetMaxSpeed(void *base, float max_speed);
static void RflBaseMotorSetPosition(void *base, float set_position);
static void RflBaseMotorSetPositionLimit(void *base, float max_position, float min_position);
static RflMotorControlMode RflBaseMotorGetMode(void *base);
static float RflBaseMotorGetSpeed(void *base);
static float RflBaseMotorGetInternalSpeed(void *base);
static float RflBaseMotorGetMaxSpeed(void *base);
static float RflBaseMotorGetPosition(void *base);
static float RflBaseMotorGetInternalPosition(void *base);
static float RflBaseMotorGetMaxPosition(void *base);
static float RflBaseMotorGetMinPosition(void *base);
static float RflBaseMotorGetTemperature(void *base);
static float RflBaseMotorGetTorque(void *base);

void RflBaseMotorGetDefaultConfig(RflBaseMotorConfig *config, RflMotorType type)
{
    memset(config, 0, sizeof(RflBaseMotorConfig));

    config->type = type;
    config->angle_format = RFL_MOTOR_ANGLE_FORMAT_CIRCLED;
    config->is_reversed = false;
    config->position_conversion_factor = 1.0f;
    config->control_period_factor = 1.0f;

    config->controller_type = RFL_MOTOR_CONTROLLER_UNDEFINED;

    config->max_position = RFL_MOTOR_DEFAULT_POSITION_RANGE;
    config->min_position = -RFL_MOTOR_DEFAULT_POSITION_RANGE;

    config->external_speed = NULL;
    config->external_position = NULL;
}

void RflBaseMotorInit(RflBaseMotor *self, RflBaseMotorConfig *config)
{
    memset(self, 0, sizeof(RflBaseMotor));

    self->type_ = config->type;
    self->mode_ = RFL_MOTOR_CONTROL_MODE_NO_FORCE;
    self->last_mode_ = RFL_MOTOR_CONTROL_MODE_NO_FORCE;
    self->angle_format_ = config->angle_format;
    self->is_reversed_ = config->is_reversed;
    self->position_conversion_factor_ = config->position_conversion_factor;
    self->control_period_factor_ = config->control_period_factor;

    self->max_position_ = config->max_position;
    self->min_position_ = config->min_position;

    self->external_speed_ = config->external_speed;
    self->external_position_ = config->external_position;

    self->SetSpeed = RflBaseMotorSetSpeed;
    self->SetMaxSpeed = RflBaseMotorSetMaxSpeed;
    self->SetPosition = RflBaseMotorSetPosition;
    self->SetPositionLimit = RflBaseMotorSetPositionLimit;
    self->GetMode = RflBaseMotorGetMode;
    self->GetSpeed = RflBaseMotorGetSpeed;
    self->GetInternalSpeed = RflBaseMotorGetInternalSpeed;
    self->GetMaxSpeed = RflBaseMotorGetMaxSpeed;
    self->GetPosition = RflBaseMotorGetPosition;
    self->GetInternalPosition = RflBaseMotorGetInternalPosition;
    self->GetMaxPosition = RflBaseMotorGetMaxPosition;
    self->GetMinPosition = RflBaseMotorGetMinPosition;
    self->GetTemperature = RflBaseMotorGetTemperature;
    self->GetTorque = RflBaseMotorGetTorque;
}

static void RflBaseMotorSetSpeed(void *base, float set_speed)
{
    RflBaseMotor *self = (RflBaseMotor *)base;
    self->set_speed_ = set_speed;
}

static void RflBaseMotorSetMaxSpeed(void *base, float max_speed)
{
    RflBaseMotor *self = (RflBaseMotor *)base;
    self->max_speed_ = max_speed;
}

static void RflBaseMotorSetPosition(void *base, float set_position)
{
    RflBaseMotor *self = (RflBaseMotor *)base;
    self->set_position_ = set_position;
}

static void RflBaseMotorSetPositionLimit(void *base, float max_position, float min_position)
{
    RflBaseMotor *self = (RflBaseMotor *)base;
    self->max_position_ = max_position;
    self->min_position_ = min_position;
}

static RflMotorControlMode RflBaseMotorGetMode(void *base)
{
    return ((RflBaseMotor *)base)->mode_;
}

static float RflBaseMotorGetSpeed(void *base)
{
    return ((RflBaseMotor *)base)->mode_;
}

static float RflBaseMotorGetInternalSpeed(void *base)
{
    return ((RflBaseMotor *)base)->mode_;
}

static float RflBaseMotorGetMaxSpeed(void *base)
{
    return ((RflBaseMotor *)base)->mode_;
}

static float RflBaseMotorGetPosition(void *base)
{
    return ((RflBaseMotor *)base)->mode_;
}

static float RflBaseMotorGetInternalPosition(void *base)
{
    return ((RflBaseMotor *)base)->mode_;
}

static float RflBaseMotorGetMaxPosition(void *base)
{
    return ((RflBaseMotor *)base)->mode_;
}

static float RflBaseMotorGetMinPosition(void *base)
{
    return ((RflBaseMotor *)base)->mode_;
}

static float RflBaseMotorGetTemperature(void *base)
{
    return ((RflBaseMotor *)base)->mode_;
}

static float RflBaseMotorGetTorque(void *base)
{
    return ((RflBaseMotor *)base)->mode_;
}
