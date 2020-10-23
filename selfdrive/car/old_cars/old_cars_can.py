# these had to be copied in because we control on bus 2

def crc8_pedal(data):
  crc = 0xFF    # standard init value
  poly = 0xD5   # standard crc8: x8+x7+x6+x4+x2+1
  size = len(data)
  for i in range(size-1, -1, -1):
    crc ^= data[i]
    for j in range(8):
      if ((crc & 0x80) != 0):
        crc = ((crc << 1) ^ poly) & 0xFF
      else:
        crc <<= 1
  return crc


def create_pedal_command(packer, gas_amount, idx):
  # Common gas pedal msg generator
  enable = gas_amount > 0.001

  values = {
    "ENABLE": enable,
    "COUNTER_PEDAL": idx & 0xF,
  }

  if enable:
    values["GAS_COMMAND"] = gas_amount * 255.
    values["GAS_COMMAND2"] = gas_amount * 255.

  dat = packer.make_can_msg("GAS_COMMAND", 2, values)[2]

  checksum = crc8_pedal(dat[:-1])
  values["CHECKSUM_PEDAL"] = checksum

  return packer.make_can_msg("GAS_COMMAND", 2, values)

def create_acc_cancel_command(packer, disengage, idx):

  values = {
    "DISABLE_REQ": disengage,
    "COUNTER_PEDAL": idx & 0xF,
  }

  dat = packer.make_can_msg("PCM_REQUEST", 0, values)[2]

  checksum = crc8_pedal(dat[:-1])
  values["CHECKSUM_PEDAL"] = checksum

  return packer.make_can_msg("PCM_REQUEST", 0, values)

def create_actuator_command(packer, actuator, amount, idx):
  # Common gas pedal msg generator
  enable = amount > 0.001

  values = {
    "ENABLE": enable,
    "COUNTER_PEDAL": idx & 0xF,
  }

  if enable:
    values["COMMAND"] = amount * 255.
    values["COMMAND2"] = amount * 255.

  dat = packer.make_can_msg(actuator, 0, values)[2]

  checksum = crc8_pedal(dat[:-1])
  values["CHECKSUM_PEDAL"] = checksum

  return packer.make_can_msg(actuator, 0, values)

def create_steer_command(packer, enable, amount, idx):
  # Common gas pedal msg generator
  
  values = {
    "ENABLE": enable,
    "EPS_TORQUE": 0,
    "COUNTER_PEDAL": idx & 0xF,
  }

  if enable:
    values["STEER_TORQUE_CMD"] = amount

  dat = packer.make_can_msg("STEER_COMMAND", 2, values)[2]

  checksum = crc8_pedal(dat[:-1])
  values["CHECKSUM_PEDAL"] = checksum

  return packer.make_can_msg("STEER_COMMAND", 2, values)  

def brake_enable(packer, enable):
  if enable:
    brake = "BRAKE_ENABLE"
    values = {
      "brake_enable_magic": 0x05cc,
    }
  else:
    brake = "BRAKE_DISABLE"
    values = {
      "brake_disable_magic": 0x05cc,
    }
  return packer.make_can_msg(brake, 2, values)

def create_brake_cmd(packer, amt):
  values = {
    "brake_command_magic": 0x05cc,
    "brake_command_pedal_request": (amt * 64),
  }
  return packer.make_can_msg("BRAKE_COMMAND", 2, values)

def create_brake_relay(packer, brake_pressure, idx):
  # Common gas pedal msg generator
  
  values = {
    "PRESSURE" : brake_pressure,
    "COUNTER_PEDAL": idx & 0xF,
  }

  dat = packer.make_can_msg("BRAKE_PRESSURE", 2, values)[2]

  checksum = crc8_pedal(dat[:-1])
  values["CHECKSUM_PEDAL"] = checksum

  return packer.make_can_msg("BRAKE_PRESSURE", 2, values)

def create_brake_actuator_command(packer, brake_pressure, brake_amount, idx):
  enable = brake_amount > 0.001

  values = {
    "ENABLE": enable,
    "PRESSURE": brake_pressure,
    "COUNTER_PEDAL": idx & 0xF,
  }

  if enable:
    values["PRESSURE_TARGET"] = brake_amount * 1024.

  dat = packer.make_can_msg("BRAKE_ACTUATOR", 2, values)[2]

  checksum = crc8_pedal(dat[:-1])
  values["CHECKSUM_PEDAL"] = checksum

  return packer.make_can_msg("BRAKE_ACTUATOR", 2, values)
