#!/bin/bash

VERSION=$(git describe --dirty --always --tags)
echo "#ifndef _VERSION_AUTOGEN_H_" > ../Yeti/Inc/version_autogen.h
echo "#define _VERSION_AUTOGEN_H_" >> ../Yeti/Inc/version_autogen.h
echo "#define VERSION_STRING \"$VERSION\"" >> ../Yeti/Inc/version_autogen.h
echo "#endif" >> ../Yeti/Inc/version_autogen.h
