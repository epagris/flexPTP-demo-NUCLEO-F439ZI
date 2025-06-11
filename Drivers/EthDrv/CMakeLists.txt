add_subdirectory(phy_drv)

if (ETH_STACK STREQUAL "ETHERLIB")
    set(ETH_DRV_SRC
        eth_drv_etherlib.c
        eth_drv_etherlib.h
        )
elseif(ETH_STACK STREQUAL "LWIP")
    set(ETH_DRV_SRC
        eth_drv_lwip.c
        eth_drv_lwip.h
    )
endif()


target_sources(
    ${CMAKE_PROJECT_NAME}
    PUBLIC

    mac_drv.c
    mac_drv.h
    
    ${ETH_DRV_SRC}
)