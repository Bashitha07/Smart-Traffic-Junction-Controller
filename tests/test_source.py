from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "assembly" / "main.asm").read_text()


def test_has_eight_traffic_states():
    assert ".equ NS_GREEN, 0" in SOURCE
    assert ".equ NS_GREEN_EXTRA, 7" in SOURCE


def test_pedestrian_interrupt_and_countdown():
    assert "int0_handler:" in SOURCE
    assert ".equ PEDESTRIAN_SECONDS, 5" in SOURCE
    assert "ldi r16, PEDESTRIAN_SECONDS" in SOURCE
    assert "ped_request:" in SOURCE


def test_night_mode_hysteresis():
    assert ".equ NIGHT_ENTER_THRESHOLD, 100" in SOURCE
    assert ".equ NIGHT_RECOVER_THRESHOLD, 350" in SOURCE
    assert ".equ NIGHT_RECOVER_SECONDS, 5" in SOURCE
    assert "set_night_outputs:" in SOURCE


def test_adc0_and_timer1_are_configured():
    assert "(1<<REFS0)" in SOURCE
    assert "TIM1_COMPAaddr" in SOURCE


if __name__ == "__main__":
    test_has_eight_traffic_states()
    test_pedestrian_interrupt_and_countdown()
    test_night_mode_hysteresis()
    test_adc0_and_timer1_are_configured()
