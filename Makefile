##############################################################################
#
#    file                 : Makefile
#    created              : Sat Jan 9 17:56:45 CET 2010
#    copyright            : (C) 2002 MasterM and MiKom
#
##############################################################################

ROBOT       = minracer
MODULE      = ${ROBOT}.so
MODULEDIR   = drivers/${ROBOT}
SOURCES     = model.cpp ${ROBOT}.cpp ${TORCS_BASE}/src/drivers/lib/ffll.a

SHIPDIR     = drivers/${ROBOT}
SHIP        = ${ROBOT}.xml car4-trb1.rgb logo.rgb
SHIPSUBDIRS = 

PKGSUBDIRS  = ${SHIPSUBDIRS}
src-robots-minracer_PKGFILES = $(shell find * -maxdepth 0 -type f -print)
src-robots-minracer_PKGDIR   = ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))

include ${MAKE_DEFAULT}
