package jolt;

enum abstract SpringMode(Int) from Int to Int {
    var FrequencyAndDamping;		///< Frequency and damping are specified
	var StiffnessAndDamping;		///< Stiffness and damping are specified
}