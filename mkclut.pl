#!/usr/bin/perl
#
# Generate a colour lookup table for LEDHat
#
# Copyright (c) 2015, 2021 Matt Evans
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#

use strict;
use Math::Trig;


my $pi = 3.141592654;

sub gammaval {
    my $i = shift;
    my $gamma = shift;
    my $f = $i/256;	# 0-1
    my $o = $f ** (1.0/$gamma);
    return 256 * $o;
}
sub sat {
    my $i = shift;
    return $i > 255 ? 255 : $i;
}

# Palette/gradient, from black to white with a red tint in-between.  Pass 0-255, ret (r,g,b)
sub palettegrad {
    my $i = shift;
    my $r;
    my $g;
    my $b;
    # Function one: just scale one colour faster than the others
    #$r = sat($i*1.4);
    #$g = sat($i);
    #$b = sat($i);
    # Function two: Truez gamma:
    $r = gammaval($i, 1.1);
    $g = gammaval($i, 0.8);
    $b = gammaval($i, 1.5);
    return ($r, $g, $b);
}

sub r_fn {
    my $i = shift;

    my $oto = $i/64.0;
    my $s = (sin( ($oto * $pi) - ($pi/2) ) + 1)/2.0;

    return $s*$i;
}

sub g_fn {
    my $i = shift;
    return gammaval($i, 0.7);
}

sub b_fn {
    my $i = shift;
    return gammaval($i, 1.4);
}

for (my $i = 0; $i < 256; $i++) {
    #    printf("%d, ", fn($i));
    my $r = r_fn($i);
    my $g = g_fn($i);
    my $b = b_fn($i);
    printf("%d,%d,%d, ", $r,$g,$b);
}
print "\n";
