package jolt;

enum abstract StateRecorderState(Int) from Int to Int {
	var None				= 0;														///< Save nothing
	var Global				= 1;														///< Save global physics system state (delta time, gravity, etc.)
	var Bodies				= 2;														///< Save the state of bodies
	var Contacts			= 4;														///< Save the state of contacts
	var Constraints			= 8;														///< Save the state of constraints
	var All					= Global | Bodies | Contacts | Constraints;					///< Save all state
}