% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%                                                                                                         %
%  This file is created as part of research done by the Multi-Agent Coordination & Control (MACC) group.  %
%                                                                                                         %
%  (C) 2013 Johan Philips, Bart Saint Germain, Jan Van Belle, Paul Valckenaers, macc.radar@gmail.com      %
%                                                                                                         %
%  Department of Mechanical Engineering, Katholieke Universiteit Leuven, Belgium.                         %
%                                                                                                         %
%  You may redistribute this software and/or modify it under either the terms of the GNU Lesser           %
%  General Public License version 2.1 (LGPLv2.1 <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>) %
%  or (at your discretion) of the Modified BSD License:                                                   %
%  Redistribution and use in source and binary forms, with or without modification, are permitted         %
%  provided that the following conditions are met:                                                        %
%     1. Redistributions of source code must retain the above copyright notice, this list of              %
%        conditions and the following disclaimer.                                                         %
%     2. Redistributions in binary form must reproduce the above copyright notice, this list of           %
%        conditions and the following disclaimer in the documentation and/or other materials              %
%        provided with the distribution.                                                                  %
%     3. The name of the author may not be used to endorse or promote products derived from               %
%        this software without specific prior written permission.                                         %
%  THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,    %
%  BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE     %
%  ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,  %
%  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS    %
%  OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY        %
%  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR           %
%  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY      %
%  OF SUCH DAMAGE.                                                                                        %
%                                                                                                         %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%% Author: bgermain
%% Created: 18-jan.-2013
%% Description: TODO: Add description to fundamental_diagram
-module(fundamental_diagram).
-include("states.hrl").
-record(k_q,{kc,c,kjam,vf,w}).

-export([init/1,init/3,kc/1,c/1,kjam/1,vf/1,q/2,u/2,w/1]).

init(Kc,C,Kjam) when ((Kc > 0) and (Kjam > Kc)) ->
	#k_q{kc=Kc,c=C,kjam=Kjam,vf=C/Kc,w=C/(Kjam-Kc)};
init(Kc,C,Kjam) ->
	util:log(info, fd, "Kc ~p > Kjam ~p!", [Kc,Kjam]),
	error.

	% using default values for C and Kjam
% C = # vehicles / second -> on highway 2100 / h -> 0.583 / s
% Kjam = # vehicles / meter -> avg vehicle length of 5 meters results in Kjam of 0.2
init({motorway,MaxSpeed,NumLanes}) when (MaxSpeed > 0) and (NumLanes > 0) ->
	C = 0.583 * NumLanes,
	init(C/MaxSpeed,C, NumLanes / ?vehicle_length);

% C = # vehicles / second -> in living street 720 / h / lane -> 0.2 / s / lane
% Kjam = # vehicles / meter -> avg vehicle length of 5 meters results in Kjam of 0.2 / lane
init({living_street, MaxSpeed, NumLanes}) when (MaxSpeed > 0) and (NumLanes > 0) ->
	C = 0.2 * NumLanes,
	init(C/MaxSpeed,C, NumLanes / ?vehicle_length);	

% C = # vehicles / second -> in service street 1080 / h / lane -> 0.3 / s / lane
% Kjam = # vehicles / meter -> avg vehicle length of 5 meters results in Kjam of 0.2 / lane
init({service, MaxSpeed, NumLanes}) when (MaxSpeed > 0) and (NumLanes > 0) ->
	C = 0.3 * NumLanes,
	init(C/MaxSpeed,C, NumLanes / ?vehicle_length);	
	
% using default values for C and Kjam for all other road types
% C = # vehicles / second -> in city 1800 / h -> 0.5 / s for each lane
% Kjam = # vehicles / meter -> avg vehicle length of 5 meters results in Kjam of 0.2 for each lane
init({RoadType,MaxSpeed,NumLanes}) when (MaxSpeed > 5) and (NumLanes > 0) ->
	C = 0.5 * NumLanes,
	init(C/MaxSpeed, C, NumLanes / ?vehicle_length);

% using default values for C and Kjam for all other road types
% C = # vehicles / second -> in city 720 / h -> 0.2 / s for each lane
% Kjam = # vehicles / meter -> avg vehicle length of 5 meters results in Kjam of 0.2 for each lane
init({RoadType,MaxSpeed,NumLanes}) when (MaxSpeed < 5) and (NumLanes > 0) ->
	C = 0.2 * NumLanes,
	init(C/MaxSpeed, C, NumLanes / ?vehicle_length).	
	
kc(#k_q{kc=Kc})->
	Kc.
c(#k_q{c=C})->
	C.
kjam(#k_q{kjam=Kjam})->
	Kjam.
vf(#k_q{vf=Vf})->
	Vf.
w(#k_q{w=W})->
	W.

q(K,#k_q{kc=Kc,c=C}) when ((K>=0) and (K=<Kc))->
	Free_Flow = line_segment:new(0, 0, Kc, C),
	line_segment:x_y(K, Free_Flow);
q(K,#k_q{kc=Kc,c=C,kjam=Kjam}) when ((K>=Kc) and (K=<Kjam))->
	Congested_Flow = line_segment:new(Kc, C, Kjam, 0),
	line_segment:x_y(K, Congested_Flow);
q(_,_)-> undefined.

u(K,#k_q{kc=Kc,c=C,kjam=_}) when ((K>=0) and (K=<Kc))->
	Free_Flow = line_segment:new(0, 0, Kc, C),
	line_segment:gradient(Free_Flow);
u(K,#k_q{kc=Kc,c=C,kjam=Kjam}) when ((K>=Kc) and (K=<Kjam))->
	Congested_Flow = line_segment:new(Kc, C, Kjam, 0),
	line_segment:gradient(Congested_Flow);
u(K,#k_q{kjam=Kjam}) when K>=Kjam-> 0.