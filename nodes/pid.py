#!/usr/bin/env python
# Copyright (c) 2012, Falkor Systems, Inc.  All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:

# Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.  Redistributions
# in binary form must reproduce the above copyright notice, this list of
# conditions and the following disclaimer in the documentation and/or
# other materials provided with the distribution.  THIS SOFTWARE IS
# PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

class Pid:
    def __init__( self, gain_p = 0.0, gain_i = 0.0, gain_d = 0.0,
                  time_constant = 0.0, limit = -1.0 ):
        self.gain_p = gain_p
        self.gain_i = gain_i
        self.gain_d = gain_d
        self.time_constant = time_constant
        self.limit = limit

        self.input = 0.0
        self.dinput = 0.0
        self.output = 0.0
        self.p = 0.0
        self.i = 0.0
        self.d = 0.0

    def update( self, new_input, x, dx, dt ):
        if dt + self.time_constant > 0.0:
            self.dinput = ( new_input - self.input ) / ( dt + self.time_constant )
            self.input = ( dt * new_input + self.time_constant * self.input ) / ( dt + self.time_constant )

        self.p = self.input - x
        self.d = self.dinput - dx
        self.i = self.i + dt * self.p

        self.output = self.gain_p * self.p + self.gain_d * self.d + self.gain_i * self.i

        if self.limit > 0.0 and abs( self.output ) > self.limit:
            if self.output < 0:
                return -self.limit
            else:
                return self.limit

        return self.output

    def reset( self ):
        self.input = self.dinput = 0.0
        self.p = self.i = self.d = 0.0

