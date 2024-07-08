% GPL - Gianluca Canzolino - Master Student of Computer Engineering - UNISA
% GPL - Giuseppe Gambardella - Master Student of Computer Engineering - UNISA
% GPL - Alberto Provenza - Master Student of Computer Engineering - UNISA

classdef soc_stm_QEP_arma < realtime.internal.SourceSampleTime ...
		& coder.ExternalDependency %...
% 		& matlab.system.mixin.Propagates ...
% 		& matlab.system.mixin.CustomIcon
	
	% SOC_STM_QEP Quadrature Encoder for STM32 Nucleo
    %
	% Configures Timer of STM32 Nucleo-F401RE hardware in encoder mode 3 to
	% decode and count quadrature encoded pulses applied on following input pins:
    %
	% TIM1
    %   PA_8 = Encoder 1 A
    %   PA_9 = Encoder 1 B
    % 
	% TIM4
    %  	PB_6 = Encoder 2 A
    %  	PB_7 = Encoder 2 B
    % 
    % The output EncoderCount is pulse count when a pulse signal comes from an optical
	% encoder mounted on a rotating machine, speed is the instant speed of the motor
	
	properties
		% Public, tunable properties.
		
	end
	
	properties (Nontunable)
		% Public, non-tunable properties.
		
        % Select Timer
        timerType (1,1) timer_stm32_arma = timer_stm32_arma.TIM_1
	end
	
	properties (Access = private)
		% Pre-computed constants.
	end
	
	methods
		% Constructor
		function obj = soc_stm_QEP(varargin)
			setProperties(obj,nargin,varargin{:});
		end
	end
	
	methods (Access=protected)
        function setupImpl(obj)
			if isempty(coder.target)
				% Place simulation setup code here
			else
				% Call C-function implementing device initialization
				coder.cinclude('soc_stm_encoder_arma.h');
                coder.ceval('initEncoder',obj.timerType);
			end
		end
		
		function [EncoderCount,Speed] = stepImpl(obj)
			EncoderCount = uint16(0);
            Speed = double(0);
			if isempty(coder.target)
				% Place simulation output code here
			else
				% Call C-function implementing device output
				EncoderCount = coder.ceval('getEncoderCount',obj.timerType);
				Speed = coder.ceval('getSpeed',obj.timerType);
			end
		end
		
		function releaseImpl(obj)
			if isempty(coder.target)
				% Place simulation termination code here
			else
				% Call C-function implementing device termination
				coder.ceval('releaseEncoder', obj.timerType);
			end
		end
	end
	
	methods (Access=protected)
		%% Define output properties
		function num = getNumInputsImpl(~)
			num = 0;
		end
		
		function num = getNumOutputsImpl(~)
			num = 2;
		end
		
		function flag = isOutputSizeLockedImpl(~,~)
			flag = true;
		end
		
		function varargout = isOutputFixedSizeImpl(~,~)
			varargout{1} = true;
			varargout{2} = true;
		end
		
% 		function flag = isOutputComplexityLockedImpl(~,~)
% 			flag = true;
% 		end
		
		function varargout = isOutputComplexImpl(~)
			varargout{1} = false;
			varargout{2} = false;
		end
		
		function varargout = getOutputSizeImpl(~)
			varargout{1} = [1,1];
			varargout{2} = [1,1];
		end
		
		function varargout = getOutputDataTypeImpl(~)
			varargout{1} = 'uint16';
            varargout{2} = 'double';
		end
		
		function icon = getIconImpl(~)
			% Define a string as the icon for the System block in Simulink.
			icon = 'STM32 NUCLEO QEP (ARMA)';
		end
	end
	
	methods (Static, Access=protected)
		function simMode = getSimulateUsingImpl(~)
			simMode = 'Interpreted execution';
		end
		
		function isVisible = showSimulateUsingImpl
			isVisible = false;
		end
		
		function sts = getSampleTimeImpl(obj)
			if isequal(obj.SampleTime, -1) || isequal(obj.SampleTime, [-1, 0])
				sts = matlab.system.SampleTimeSpecification('Type', 'Inherited');
			elseif isequal(obj.SampleTime, [0, 1])
				sts = matlab.system.SampleTimeSpecification('Type', 'Fixed In Minor Step');
			else
				if numel(obj.SampleTime) == 1
					sampleTime = obj.SampleTime;
					offset = 0;
				else
					sampleTime = obj.SampleTime(1);
					offset = obj.SampleTime(2);
				end
				sts = matlab.system.SampleTimeSpecification('Type', 'Discrete', ...
					'SampleTime', sampleTime, 'Offset', offset);
			end
		end
    end
    
    methods(Static, Access=protected)
        function header = getHeaderImpl()
            header = matlab.system.display.Header(mfilename('class'),...
                'ShowSourceLink', false, ...
                'Title','STM32F401RE_Encoder', ...
                'Text', ['Quadrature Encoder for STM32 Nucleo.' newline newline ...
                'Nucleo-F401RE hardware in encoder mode 3 to ' ...
                'decode and count quadrature encoded pulses applied on following input pins:' newline newline ...
                'TIM1' newline...
                '       PA_8 = Encoder A' newline ...
                '       PA_9 = Encoder B' newline newline...
                'TIM4' newline...
                '       PB_6 = Encoder A' newline ...
                '       PB_7 = Encoder B' newline newline...
                'The output EncoderCount is pulse count when a pulse signal comes from an optical ' ...
                'encoder mounted on a rotating machine, speed is the instant speed of the motor' newline newline...
                'Gianluca Canzolino - Master Student of Computer Engineering - UNISA' newline...
                'Giuseppe Gambardella - Master Student of Computer Engineering - UNISA' newline...
                'Alberto Provenza - Master Student of Computer Engineering - UNISA']);
        end
    end
	
	methods (Static)
		function name = getDescriptiveName()
			name = 'STM32 NUCLEO QEP (ARMA)';
		end
		
		function b = isSupportedContext(context)
			b = context.isCodeGenTarget('rtw');
		end
		
		function updateBuildInfo(buildInfo, context)
			if context.isCodeGenTarget('rtw')
				% Update buildInfo
				srcDir = fullfile(fileparts(mfilename('fullpath')),'src');
				includeDir = fullfile(fileparts(mfilename('fullpath')),'include');
				addIncludePaths(buildInfo,includeDir);
				addSourceFiles(buildInfo,'soc_stm_encoder_arma.cpp',srcDir);
			end
		end
	end
end

% LocalWords:  ZI
