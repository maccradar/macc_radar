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
%% Created: 17-jan.-2013
%% Description: TODO: Add description to line_segment
-module(line_segment).
-include("consts.hrl").
-include("debug.hrl").
-export([gradient/1,test/0,x_y/2,y_x/2,new/4,split_x/3,update/3,get/2,join/2,intersection/2,intersection/4,x_y/4,y_x/4,is_parallel/2,steeper/2,gentler/2]).
-export_type([line_segment/0]).


-record(segment,{p1::{number(),number()}
				,p2::{number(),number()}
				,gradient=?not_initialised::?not_initialised|number()}).
-type line_segment() :: #segment{}.

is_parallel(#segment{p1={X1,Y1},p2={X2,Y2}},G) when ((Y2-Y1)==((X2-X1) * G))->
	true;
is_parallel(#segment{p1={_,_},p2={_,_}},_)->false;
is_parallel(_, _) -> ?undefined.
steeper(#segment{p1={X1,Y1},p2={X2,Y2}},G) when (abs(Y2-Y1)>(abs(X2-X1) * G))->
	true;
steeper(#segment{p1={_,_},p2={_,_}},_)->
	false;
steeper(_, _) -> ?undefined.
gentler(#segment{p1={X1,Y1},p2={X2,Y2}},G) when (abs(Y2-Y1)<(abs(X2-X1) * G))->
	true;
gentler(#segment{p1={_,_},p2={_,_}},_)->
	false;
gentler(_, _) -> ?undefined.

-spec gradient(LS)->G when
						LS :: line_segment(),
						G :: number().
gradient(#segment{p1={X1,Y1},p2={X2,Y2},gradient=?not_initialised}) when X1=/=X2->
	(Y2-Y1)/(X2-X1);
gradient(#segment{gradient=?not_initialised})->
	?undefined;
gradient(#segment{gradient=G})->
	G;
gradient(_)->
	?undefined.

x_y(X,X1,Y1,G)->
	(G*(X-X1)) +Y1.
x_y(X,#segment{p1={X1,Y1}}) when X== X1->
	Y1;
x_y(X,#segment{p2={X1,Y1}}) when X== X1->
	Y1;
x_y(X,S = #segment{p1={X1,Y1},p2={X2,_}}) when ((X1<X) and (X<X2))->
	G = gradient(S),
	x_y(X, X1, Y1, G);
x_y(_,_)->
	?undefined.
y_x(Y,X0,Y0,G)->
	((Y-Y0)/G) +X0.
y_x(Y, #segment{p1={X1,Y1}}) when Y == Y1->
	X1;
y_x(Y,#segment{p2={X1,Y1}}) when Y == Y1->
	X1;
y_x(Y,S = #segment{p1={X1,Y1},p2={_,Y2}}) when ((Y1=<Y) and (Y=<Y2))->
	G = gradient(S),
	((1/G)*(Y-Y1)) +X1;
y_x(_, _)->
	?undefined.

-spec new(X1,Y1,X2,Y2)->S when
							X1 :: number(),
							Y1 :: number(),
							X2 :: number(),
							Y2 :: number(),
							S :: line_segment().
new(X1,Y1,X2,Y2)->
	S = #segment{p1={X1,Y1},p2={X2,Y2}},
	S#segment{gradient = gradient(S)}.


split_x(#segment{p1={X1,Y1},p2={X2,Y2}},X,Y) when ((X1<X)and(X<X2))->
	{new(X1, Y1, X, Y),new(X, Y, X2, Y2)};
split_x(S, _, _)->
	S.

-spec join(S1,S2) -> S when %% joins two adjoint segements, taking the start point of S1 and the end point of S2
						 S1 :: nil|line_segment(), 					%% segment 1
						 S2 :: nil|line_segment(), 					%% segment 2
						 S :: line_segment() | ?inconsistent. 	%% resulting segment

join(nil,#segment{p1=P,p2={X2,Y2}})->
	#segment{p1=P,p2={X2,Y2}};
join(#segment{p1=P,p2={X2,Y2}},nil)->
	#segment{p1=P,p2={X2,Y2}};
join(#segment{p1={X1,Y1},p2=P2},#segment{p1=P1,p2={X2,Y2}}) when P1 == P2->
	new(X1, Y1, X2, Y2);
join(_S1,_S2)->
	?inconsistent.

test()->
	gradient(#segment{p1={2,2},p2={5,4}}),
	S1 = new(0, 0, 4, 4),
	S2 = new(0, 4, 4, 0),
	intersection( S2, 0, 0, gradient(S1)).

update(x1,New,S = #segment{p1={_,Y}})->SU = S#segment{p1={New,Y}},SU#segment{gradient=gradient(SU)};
update(y1,New,S = #segment{p1={X,_}})->SU = S#segment{p1={X,New}},SU#segment{gradient=gradient(SU)};
update(x2,New,S = #segment{p2={_,Y}})->SU = S#segment{p2={New,Y}},SU#segment{gradient=gradient(SU)};
update(y2,New,S = #segment{p2={X,_}})->SU = S#segment{p2={X,New}},SU#segment{gradient=gradient(SU)}.

-spec get(Info,S)->V when
					   Info :: 'x1'|'y1'|'x2'|'y2'|'gradient',	%% the information you want to receive
					   S :: line_segment(),						%% the line segement you want to query
					   V :: number().							%% the information
get(x1,#segment{p1={V,_}})->V;
get(y1,#segment{p1={_,V}})->V;
get(x2,#segment{p2={V,_}})->V;
get(y2,#segment{p2={_,V}})->V;
get(gradient,#segment{gradient=G})->G.

intersection(_,nil)->
	{?undefined,?undefined};
intersection(nil,_)->
	{?undefined,?undefined};
intersection(S1,#segment{p1={X21,Y21},p2={X22,Y22},gradient=G2})->
	case intersection(S1, X21, Y21, G2) of
		{_,?undefined} -> {?undefined,?undefined};
		{X,Y} when ((X21 =< X) and (X =< X22) and (Y21 =< Y) and (Y =< Y22)) -> {X,Y};
		_ -> {?undefined,?undefined}
	end.
	

-spec intersection(S,X,Y,G)->P when %% intersection between segment and line represented by a point and a gradient
								 S :: line_segment(), 		%% the segement 
								 X :: number(),				%% the x value of the point representing the line
								 Y :: number(),				%% the y value of the point representing the line
								 G :: number(),				%% the gradient of the line
								 P :: {number(),number()}|undefined. 	%% the intersection point
								 
intersection(#segment{p1={X1,Y1},p2={X1,Y1}},X21,Y21,G2) when ((G2 * (X1-X21)) == (Y1-Y21)   )->
	{X1,Y1};
intersection(S1=#segment{},X21,Y21,G2)->
	Y11 = get(y1, S1),
	X11 = get(x1, S1),
	G1 = gradient(S1),
	case ((G1 =/= G2) and (G1 =/=?undefined) and (G2 =/= ?undefined)) of
		true -> A2 = Y21-G2*X21,
				A1 = Y11-G1*X11,	
				X = (A2 - A1)/(G1-G2),
				Y = x_y(X, S1),
				{X,Y};
		false -> {?undefined,?undefined}
	end.


	

