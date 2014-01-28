##############################################################################
#
#    file                 : Makefile
#    created              : Wed Dec 11 16:54:59 CET 2013
#    copyright            : (C) 2002 Dizan Vasquez
#
##############################################################################

ROBOT       = trosc
MODULE      = ${ROBOT}.so
MODULEDIR   = drivers/${ROBOT}
SOURCES     = ${ROBOT}.cpp facade.cpp semaphore.cpp DataCollection.cpp

SHIPDIR     = drivers/${ROBOT}
SHIP        = ${ROBOT}.xml car8-trb1.rgb logo.rgb
SHIPSUBDIRS = 

PKGSUBDIRS  = ${SHIPSUBDIRS}
src-robots-trosc_PKGFILES = $(shell find * -maxdepth 0 -type f -print)
src-robots-trosc_PKGDIR   = ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))

include ${MAKE_DEFAULT}
