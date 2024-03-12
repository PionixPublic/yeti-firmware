#ifndef SRC_EVDRIVERS_CPSTATE_H_
#define SRC_EVDRIVERS_CPSTATE_H_

enum class InternalCPState {
	Disabled, A, B, C, D, E, F, DF
};

inline CpState to_CpState(InternalCPState input_state) {
	CpState output_state;

	switch (input_state) {
	case InternalCPState::A:
		output_state = CpState::CpState_STATE_A;
		break;
	case InternalCPState::B:
		output_state = CpState::CpState_STATE_B;
		break;
	case InternalCPState::C:
		output_state = CpState::CpState_STATE_C;
		break;
	case InternalCPState::D:
		output_state = CpState::CpState_STATE_D;
		break;

	case InternalCPState::E:
		output_state = CpState::CpState_STATE_E;
		break;
	case InternalCPState::F:
		output_state = CpState::CpState_STATE_F;
		break;
	default:
		output_state = CpState::CpState_STATE_F;
		break;
	}

	return output_state;
}
#endif // SRC_EVDRIVERS_CPSTATE_H_
