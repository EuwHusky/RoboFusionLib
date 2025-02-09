#include "stdlib.h"

#include "dev_base_motor.h"

static RflResult RflBaseMotorSetSpeed(void *base, float set_speed);
static RflResult RflBaseMotorSetMaxSpeed(void *base, float max_speed);
static RflResult RflBaseMotorSetPosition(void *base, float set_position);
static RflResult RflBaseMotorSetPositionLimit(void *base, float max_position, float min_position);
static RflResult RflBaseMotorGetMode(void *base);
static RflResult RflBaseMotorGetSpeed(void *base);
static RflResult RflBaseMotorGetInternalSpeed(void *base);
static RflResult RflBaseMotorGetMaxSpeed(void *base);
static RflResult RflBaseMotorGetPosition(void *base);
static RflResult RflBaseMotorGetInternalPosition(void *base);
static RflResult RflBaseMotorGetMaxPosition(void *base);
static RflResult RflBaseMotorGetMinPosition(void *base);
static RflResult RflBaseMotorGetTemperature(void *base);
static RflResult RflBaseMotorGetTorque(void *base);

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

static RflResult RflBaseMotorSetSpeed(void *base, float set_speed)
{
    RflResult ret = {0};
    if (base == NULL)
    {
        ret.error = RFL_ERROR_NULL_POINTER;
        return ret;
    }

    RflBaseMotor *self = (RflBaseMotor *)base;
    self->set_speed_ = set_speed;

    ret.error = RFL_SUCCESS;

    return ret;
}

static RflResult RflBaseMotorSetMaxSpeed(void *base, float max_speed)
{
    RflResult ret = {0};
    if (base == NULL)
    {
        ret.error = RFL_ERROR_NULL_POINTER;
        return ret;
    }
    if (max_speed < 0.0f)
    {
        ret.error = RFL_ERROR_INVALID_ARG;
        return ret;
    }

    RflBaseMotor *self = (RflBaseMotor *)base;
    self->max_speed_ = max_speed;

    ret.error = RFL_SUCCESS;

    return ret;
}

static RflResult RflBaseMotorSetPosition(void *base, float set_position)
{
    RflResult ret = {0};
    if (base == NULL)
    {
        ret.error = RFL_ERROR_NULL_POINTER;
        return ret;
    }

    RflBaseMotor *self = (RflBaseMotor *)base;

    if (set_position > self->max_position_ || set_position < self->min_position_)
    {
        ret.error = RFL_ERROR_INVALID_ARG;
        return ret;
    }

    self->set_position_ = set_position;

    ret.error = RFL_SUCCESS;

    return ret;
}

static RflResult RflBaseMotorSetPositionLimit(void *base, float max_position, float min_position)
{
    RflResult ret = {0};
    if (base == NULL)
    {
        ret.error = RFL_ERROR_NULL_POINTER;
        return ret;
    }

    RflBaseMotor *self = (RflBaseMotor *)base;
    self->max_position_ = max_position;
    self->min_position_ = min_position;

    ret.error = RFL_SUCCESS;

    return ret;
}

static RflResult RflBaseMotorGetMode(void *base)
{
    RflResult ret = {0};
    if (base == NULL)
    {
        ret.error = RFL_ERROR_NULL_POINTER;
        return ret;
    }

    ret.value.u = (uint32_t)((RflBaseMotor *)base)->mode_;
    ret.error = RFL_SUCCESS;

    return ret;
}

static RflResult RflBaseMotorGetSpeed(void *base)
{
    RflResult ret = {0};
    if (base == NULL)
    {
        ret.error = RFL_ERROR_NULL_POINTER;
        return ret;
    }

    ret.value.f = ((RflBaseMotor *)base)->speed_;
    ret.error = RFL_SUCCESS;

    return ret;
}

static RflResult RflBaseMotorGetInternalSpeed(void *base)
{
    RflResult ret = {0};
    if (base == NULL)
    {
        ret.error = RFL_ERROR_NULL_POINTER;
        return ret;
    }

    ret.value.f = ((RflBaseMotor *)base)->internal_speed_;
    ret.error = RFL_SUCCESS;

    return ret;
}

static RflResult RflBaseMotorGetMaxSpeed(void *base)
{
    RflResult ret = {0};
    if (base == NULL)
    {
        ret.error = RFL_ERROR_NULL_POINTER;
        return ret;
    }

    ret.value.f = ((RflBaseMotor *)base)->max_speed_;
    ret.error = RFL_SUCCESS;

    return ret;
}

static RflResult RflBaseMotorGetPosition(void *base)
{
    RflResult ret = {0};
    if (base == NULL)
    {
        ret.error = RFL_ERROR_NULL_POINTER;
        return ret;
    }

    ret.value.f = ((RflBaseMotor *)base)->position_;
    ret.error = RFL_SUCCESS;

    return ret;
}

static RflResult RflBaseMotorGetInternalPosition(void *base)
{
    RflResult ret = {0};
    if (base == NULL)
    {
        ret.error = RFL_ERROR_NULL_POINTER;
        return ret;
    }

    ret.value.f = ((RflBaseMotor *)base)->internal_position_;
    ret.error = RFL_SUCCESS;

    return ret;
}

static RflResult RflBaseMotorGetMaxPosition(void *base)
{
    RflResult ret = {0};
    if (base == NULL)
    {
        ret.error = RFL_ERROR_NULL_POINTER;
        return ret;
    }

    ret.value.f = ((RflBaseMotor *)base)->max_position_;
    ret.error = RFL_SUCCESS;

    return ret;
}

static RflResult RflBaseMotorGetMinPosition(void *base)
{
    RflResult ret = {0};
    if (base == NULL)
    {
        ret.error = RFL_ERROR_NULL_POINTER;
        return ret;
    }

    ret.value.f = ((RflBaseMotor *)base)->min_position_;
    ret.error = RFL_SUCCESS;

    return ret;
}

static RflResult RflBaseMotorGetTemperature(void *base)
{
    RflResult ret = {0};
    if (base == NULL)
    {
        ret.error = RFL_ERROR_NULL_POINTER;
        return ret;
    }

    ret.value.f = ((RflBaseMotor *)base)->temperature_;
    ret.error = RFL_SUCCESS;

    return ret;
}

static RflResult RflBaseMotorGetTorque(void *base)
{
    RflResult ret = {0};
    if (base == NULL)
    {
        ret.error = RFL_ERROR_NULL_POINTER;
        return ret;
    }

    ret.value.f = ((RflBaseMotor *)base)->torque_;
    ret.error = RFL_SUCCESS;

    return ret;
}
