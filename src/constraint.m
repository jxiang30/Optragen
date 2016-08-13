%=============================== constraint ==============================
%
% @class    constraint
%
% @brief    Implements abstraction of an optimal control constraint.
%
% 
%=============================== constraint ==============================

%
% @file     constraint.m
%
% @author   Raktim Bhattacharya,        raktim@ae.tamu.edu  [author]
%           Patricio A. Vela,           pvela@gatech.edu    [modify]
% @date     2004/XX/XX [created]
%
% @note
%   indent is 2 spaces.
%   tab is aligned- and converted to- 4 spaces.
%
% Original text in header:
%   Nonlinear Path Planning Toolbox v 1.0
%   Copyright (c) 2004 by                
%   Raktim Bhattacharya, (raktim@cds.caltech.edu)
%   California Institute of Technology               
%   Control and Dynamical Systems 
%   All right reserved.                
%
%=============================== constraint ==============================
classdef constraint < handle

%============================ Member Variables ===========================
%

properties
  func;                 %! Structure containing constraint information.
  type;                 %! Type of constraint.
end

%)
%
%============================= Public Methods ============================
%
%(

methods

  %============================= constraint ============================
  %
  % @brief      Define a constraint for inclusion into optimal control problems.
  %
  % This function defines the initial cost function
  % func definition can be either
  %           a) Character string containing the function
  %           b) mFunction object
  %           c) cFunction object
  %
  %TODO: Should move different types to their own classes?
  %
  function this = constraint(lb,func,ub,type)

  %================== Error Checking for Input Data ==================
  if nargin~=4
    error('Usage: constrObj = constraint(lb,func,ub,type);');
  end

  if ~isa(lb,'numeric')
    error('Expecting numeric array for lower bound');
  end

  if ~isa(ub,'numeric')
    error('Expecting numeric array for upper bound');
  end

  if ~isa(type,'char')
    error('Expecting character array for type');
  end

  TYPES = {'initial','trajectory','final','galerkin'};
  ii = strmatch(lower(type),TYPES,'exact');
  if isempty(ii)
    error(['Constraint <type> must be one of the following: ' ...
           'initial, trajectory, final, galerkin']);
  end

  %===================================================================

  this.type = lower(type); 

  if isa(func,'char')
    constr = this.constraint_char(lb,func,ub);
  elseif isa(func,'mFunction')
    constr = this.constraint_mFunction(lb,func,ub);
  elseif isa(func,'cFunction')
    constr = this.constraint_cFunction(lb,func,ub);
  else
    error('Illegal data type in first argument.');
  end

  this.func = constr; 

  end

  
  %================================ get ================================
  %
  % @brief  Get asset properties from the specified object and return value.
  %
  %
  function val = get(a,prop_name)

  switch prop_name
    case 'func'
      val = a.func;
    case 'type'
      val = a.type;
    case 'tag'
      val = a.tag;
    otherwise
      error([prop_name,' Is not a valid asset property'])
  end

  end

  %================================ set ================================
  %
  % @brief  Set asset properties and return the updated object
  %
  %
  function a = set(a,varargin)
  
  property_argin = varargin;
  disp(['Cannot alter attributes of a constraint object. '
        'Create a new constraint object instead']);

  end


  %================================ plus ===============================
  %
  % @brief  Concatenate constraints.
  %
  function T = plus(T1,T2)
  
  [m1,n1] = size(T1);
  [m2,n2] = size(T2);
  
  if (m1~=1 | m2~=1)
      error('Arguments must be row vectors');
  end
  
  T = [T1 T2];

  end


end

%)
%
%=========================== Protected Methods ===========================
%
%(

methods(Access = protected)


  %======================== constraint_cFunction =======================
  %
  % @brief  Defines constraint function, defined as a character array.
  %
  %TODO: Above description might be wrong.
  %
  function constr = constraint_cFunction(this, lb,func,ub)

  % Get all the trajectories from the workspace

  N = get(func,'nFunc');

  if length(lb) ~= N | length(ub) ~= N
    error('Bounds must be scalars for analytical constraint functions');
  end

  % Check signature list of the cFunction
  % It should match that required by nonlinear constraints

  constr.grad = [];
  constr.func = func;
  constr.Tnames = get(func,'varList');
  constr.lb = scaleInf(lb);
  constr.ub = scaleInf(ub);

  end


  %========================== constraint_char ==========================
  %
  % @brief  Defines constraint function, defined as a character array.
  %
  function constr = constraint_char(this, lb,func,ub)
  
  % Get all the trajectories from the workspace
  
  if length(lb) > 1 | length(ub) > 1
    error('Bounds must be scalars for analytical constraint functions');
  end
  
  varnames = symvar(func);
  
  Tnames = getWorkSpaceTrajNames;
  
  fvars = [];
  for i=1:length(varnames)
    I = strcmp(varnames{i},Tnames);
    if sum(I)>0 
      fvars = [fvars,{varnames{i}}];
    end
  end
  
  grad = optrautil.getGradient(func,fvars);   % Char cell array
  
  constr.grad = grad;
  constr.func = func;
  constr.Tnames = fvars;
  constr.lb = scaleInf(lb);
  constr.ub = scaleInf(ub);

  end

end

%)
%
%=========================================================================

end
