#!/bin/sh

set -e

name="cc16"

N=0
dir="/lib/modules/`uname -r`/kernel/drivers/char/"
dst="$dir/$name.ko"

if [ -f "$dst" ] ; then
	bak="$dir/$name.$N"

	while [ -f "$bak" ] ; do
		N=$(( $N + 1 ))
		bak="$dir/$name.$N"
	done

	mv $dst $bak
	echo "Old module: moved to $bak."
fi

echo "New module installed as $dst."
install -m644 $name.ko $dst

echo "Removing old module."
rmmod cc16 || echo "Could not remove module from running kernel..."

echo "Inserting new module."
modprobe -f cc16 io=0x280 irq=5
