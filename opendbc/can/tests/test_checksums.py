from opendbc.can.parser import CANParser
from opendbc.can.packer import CANPacker


class TestCanChecksums:

  def verify_checksum(self, subtests, dbc_file: str, msg_name: str, msg_addr: int, test_messages: list[bytes],
                      checksum_field: str = 'CHECKSUM', counter_field = 'COUNTER'):
    """
    Verify that opendbc calculates payload CRCs/checksums matching those received in known-good sample messages
    Depends on all non-zero bits in the sample message having a corresponding DBC signal, add UNKNOWN signals if needed
    """
    parser = CANParser(dbc_file, [(msg_name, 0)], 0)
    packer = CANPacker(dbc_file)

    for data in test_messages:
      expected_msg = (msg_addr, data, 0)
      parser.update_strings([0, [expected_msg]])
      expected = copy.deepcopy(parser.vl[msg_name])

      modified = copy.deepcopy(expected)
      modified.pop(checksum_field, None)
      modified_msg = packer.make_can_msg(msg_name, 0, modified)

      parser.update_strings([0, [modified_msg]])
      tested = parser.vl[msg_name]
      with subtests.test(counter=expected[counter_field]):
        assert tested[checksum_field] == expected[checksum_field]

  def verify_fiat_fastback_crc(self, subtests, msg_name: str, msg_addr: int, test_messages: list[bytes]):
    """Test modified SAE J1850 CRCs, with special final XOR cases for EPS messages"""
    assert len(test_messages) >= 3
    self.verify_checksum(subtests, "fca_fastback_limited_edition_2024_generated", msg_name, msg_addr, test_messages)

  def test_fiat_fastback_das_1(self, subtests):
    self.verify_fiat_fastback_crc(subtests, "DAS_1", 0x2FA, [
      b'\x02\x0E\x80\x00',
      b'\x20\x09\x3E\x00',
      b'\x00\x01\xA3\x00',
      b'\x00\x04\x52\x00',
    ])

  def test_fiat_fastback_lka_command(self, subtests):
    self.verify_fiat_fastback_crc(subtests, "LKAS_COMMAND", 0x1F6, [
      b'\x80\x00\x03\x16',
      b'\x80\x00\x00\x31',
      b'\x80\x00\x08\xD9',
    ])

  def verify_fca_giorgio_crc(self, subtests, msg_name: str, msg_addr: int, test_messages: list[bytes]):
    """Test modified SAE J1850 CRCs, with special final XOR cases for EPS messages"""
    assert len(test_messages) == 3
    self.verify_checksum(subtests, "fca_giorgio", msg_name, msg_addr, test_messages)

  def test_fca_giorgio_eps_1(self, subtests):
    self.verify_fca_giorgio_crc(subtests, "EPS_1", 0xDE, [
      b'\x17\x51\x97\xcc\x00\xdf',
      b'\x17\x51\x97\xc9\x01\xa3',
      b'\x17\x51\x97\xcc\x02\xe5',
    ])

  def test_fca_giorgio_eps_2(self, subtests):
    self.verify_fca_giorgio_crc(subtests, "EPS_2", 0x106, [
      b'\x7c\x43\x57\x60\x00\x00\xa1',
      b'\x7c\x63\x58\xe0\x00\x01\xd5',
      b'\x7c\x63\x58\xe0\x00\x02\xf2',
    ])

  def test_fca_giorgio_eps_3(self, subtests):
    self.verify_fca_giorgio_crc(subtests, "EPS_3", 0x122, [
      b'\x7b\x30\x00\xf8',
      b'\x7b\x10\x01\x90',
      b'\x7b\xf0\x02\x6e',
    ])

  def test_fca_giorgio_abs_2(self, subtests):
    self.verify_fca_giorgio_crc(subtests, "ABS_2", 0xFE, [
      b'\x7e\x38\x00\x7d\x10\x31\x80\x32',
      b'\x7e\x38\x00\x7d\x10\x31\x81\x2f',
      b'\x7e\x38\x00\x7d\x20\x31\x82\x20',
    ])

  def test_honda_checksum(self):
    """Test checksums for Honda standard and extended CAN ids"""
    dbc_file = "honda_accord_2018_can_generated"
    msgs = [("LKAS_HUD", 0), ("LKAS_HUD_A", 0)]
    parser = CANParser(dbc_file, msgs, 0)
    packer = CANPacker(dbc_file)

    values = {
      'SET_ME_X41': 0x41,
      'STEERING_REQUIRED': 1,
      'SOLID_LANES': 1,
      'BEEP': 0,
    }

    # known correct checksums according to the above values
    checksum_std = [11, 10, 9, 8]
    checksum_ext = [4, 3, 2, 1]

    for std, ext in zip(checksum_std, checksum_ext):
      msgs = [
        packer.make_can_msg("LKAS_HUD", 0, values),
        packer.make_can_msg("LKAS_HUD_A", 0, values),
      ]
      parser.update_strings([0, msgs])

      assert parser.vl['LKAS_HUD']['CHECKSUM'] == std
      assert parser.vl['LKAS_HUD_A']['CHECKSUM'] == ext
