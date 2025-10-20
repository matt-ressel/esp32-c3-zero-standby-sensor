# CMakeLists.txt for ESP-NOW sender component
# This component handles ESP-NOW initialization and data transmission.
idf_component_register(SRCS "espnow_sender.c" INCLUDE_DIRS "." PRIV_REQUIRES esp_wifi)



# If a gateway MAC address is configured in Kconfig, process it and pass it to the compiler
if(CONFIG_GATEWAY_MAC)
	# 1. Get the MAC address string from Kconfig (CONFIG_GATEWAY_MAC)
	# Example format: "AA:BB:CC:DD:EE:FF"
	set(MAC_STRING "${CONFIG_GATEWAY_MAC}")

	# 2. Replace all colons ":" with ",0x"
	string(REPLACE ":" ",0x" MAC_AS_LIST "${MAC_STRING}")

	# 3. Add "0x" at the beginning of the string
	# Resulting format: "0xAA,0xBB,0xCC,0xDD,0xEE,0xFF"
	set(MAC_AS_LIST "0x${MAC_AS_LIST}")

	# 4. Pass the processed string as a macro definition to the C compiler
	# The macro will have the form e.g. -DGATEWAY_MAC_AS_BYTE_ARRAY=0xAA,0xBB,0xCC,0xDD,0xEE,0xFF
	target_compile_definitions(${COMPONENT_TARGET} PRIVATE "-DGATEWAY_MAC_AS_BYTE_ARRAY=${MAC_AS_LIST}")
endif()