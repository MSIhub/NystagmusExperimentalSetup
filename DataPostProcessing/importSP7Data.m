function SP7MotionData = importSP7Data(filename, dataLines)
%IMPORTFILE Import data from a text file
%  SP7MOTIONDATA = IMPORTFILE(FILENAME) reads data from
%  text file FILENAME for the default selection.  Returns the data as a
%  table.
%
%  SP7MOTIONDATA = IMPORTFILE(FILE, DATALINES) reads data
%  for the specified row interval(s) of text file FILENAME. Specify
%  DATALINES as a positive scalar integer or a N-by-2 array of positive
%  scalar integers for dis-contiguous row intervals.
%
%  Example:
%  SP7MotionData = importfile("C:\OneDrive\OneDrive - unige.it\OthersWork\SofiaNystagmus\DataLog\synctest\SP7MotionData-20220601183017.log", [2, Inf]);
%
%  See also READTABLE.


%% Input handling

% If dataLines is not specified, define defaults
if nargin < 2
    dataLines = [2, Inf];
end

%% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 42);

% Specify range and delimiter
opts.DataLines = dataLines;
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["Timestamp", "q1", "qdot1", "qref1", "qdotref1", "q2", "qdot2", "qref2", "qdotref2", "q3", "qdot3", "qref3", "qdotref3", "q4", "qdot4", "qref4", "qdotref4", "q5", "qdot5", "qref5", "qdotref5", "q6", "qdot6", "qref6", "qdotref6", "q7", "qdot7", "qref7", "qdotref7", "p1", "pdot1", "p2", "pdot2", "p3", "pdot3", "p4", "pdot4", "p5", "pdot5", "p6", "pdot6", "EpochTimeStampMS"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
SP7MotionData = readtable(filename, opts);

end