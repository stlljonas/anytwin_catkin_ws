function setupPath(varargin)

if (nargin == 0)
    homepath = getenv('HOME');    
    gitpath = [homepath '/git'];
elseif (nargin == 1)
    gitpath = varargin{1};
else 
    error('1 or 0 inputs are required.');
end

addpath(genpath([gitpath '/kindr/matlab']));
addpath(genpath([gitpath '/anymal_common/anymal_logging/signal_logger']));
addpath(genpath([gitpath '/signal_logger/signal_logger_std/matlab']));
end

