clear
clc
pause (3)
format shortG

%% Communication with the device and data collection
arduino=serialport("COM4", 921600);
Ns=8*51; % Total number of samples
Np=8; % Pulse number
configureTerminator(arduino, "LF");
flag =0;
while (flag == 0)
    line = readline(arduino);
    if strcmp(line, "Ns: ")
        % Obtain number of samples
        Ns = str2double(extractAfter(line, "Ns: "));
        % Initialize data vectors
        bufferI = zeros(1, Ns);
        bufferQ = zeros(1, Ns);
        bufferT = zeros(1, Np);
    elseif startsWith (line, "bufferI: ")
        % Get bufferI data
        dataI = str2double(split(extractAfter(line, "bufferI: "), " "));
        bufferI = dataI;
    elseif startsWith(line, "bufferQ: ")
        % Get bufferQ data
        dataQ = str2double(split(extractAfter(line, "bufferQ: "), " "));
        bufferQ = dataQ;
    elseif startsWith(line, "Tiempo: ")
        % Get bufferT data
        dataT = str2double(split(extractAfter(line, "Tiempo: "), " "));
        bufferT = dataT;
        flag = 1;
        
    end
end
% Exit loop when complete data has been received
if length(bufferI)
end

%% Transformation and reorganization of the data for the case study
Q = dataQ(1:Ns+1);
I = dataI(1:Ns+1);
MatrizIQ = I+j*Q;
% We eliminate the final NaN
MatrizIQ = MatrizIQ(1:end-1);
MatrizT = dataT(1:end-1);

% First reshape the matrix to 8x51 (Np x Number of samples for each pulse)
filas_nuevas = Np;
columnas_nuevas = (Ns/Np);
MatrizRemodelada = reshape(MatrizIQ, filas_nuevas, columnas_nuevas);
% Invert the order of the rows
MatrizIQ = flipud(MatrizIQ);
% Calculates the mean of each row
media_por_filas = mean(MatrizRemodelada, 2); 
% Removes the mean by rows
MatrizSinOffset = bsxfun(@minus, MatrizRemodelada, media_por_filas);

%% Declaration of variables required for the following steps
% We define the parameters necessary for the following steps of the algorithm
fs=100e6;
v=4;
c=3e8;
BW = 240e6;
% We need the Sweep Length that we have in dataT to facilitate the calculations
% we are going to measure all the data except the first one that
%showed us an absolute time and not a time difference.
sweepLenght= mean(MatrizT(2:end-1));
% Data is stored in microseconds and then converted to seconds.
sweepLenght= sweepLenght*1e-6;
% Center frequency of the sweep
f0=24.005e9;
fc=f0+(BW/2);
% We need the sweep rate
sweepRate = BW/sweepLenght;
% Position difference between the different pulses
delta_x=sweepLenght*v;
% Create vector kx
kx = linspace(-pi/delta_x, pi/delta_x, columnas_nuevas);
% Create vector dkr
dkr = linspace((4*pi/c)*(-BW/2), (4*pi/c)*(BW/2), columnas_nuevas);
% Create vector kr
kr = (4*pi/c)*fc + dkr;
% Calculate ky0
ky0 = sqrt(kr(1)^2 - kx(1)^2);
% Calculate ky spacing after interpolation
ky_delta = kr(2) - kr(1);
% Create vector ky_interp after interpolation
ky_interp = ky0:ky_delta:kr(end);

% Number of zeros to add symmetrically
zpad = 10; 
% Filling zeros symmetrically in the azimuth direction
MatrizSinOffset = padarray(MatrizSinOffset, [zpad/2, 0], 0, 'both');

%% First step of the algorithm
% Applies the Fourier Transform to columns
nfft = 51;  % Fourier transform size
img = fft(MatrizSinOffset, [], 2);

% A representation of the first step of the algorithm after FFT
figure (1)
imagesc(abs(img(:, 1:51)));
xlabel('Distance');
ylabel('Pulse Number');
title('Representation after the first steps of the algorithm');
colorbar;



%% Second step of the algorithm
% Calculate RFM filter
mf = exp(1i * (dkr(:).' .* kx(:)) .* (c * v / (4 * pi * sweepRate)));
% Apply RFM Filter
img = img * mf';

figure (2)
imagesc(abs(img(:, 1:51)));
xlabel('Distance');
ylabel('Pulse Number');
title('Representation after the second steps of the algorithm');
colorbar;
%% Third step of the algorithm
% Calculate the original grid ky 
R = linspace(0, 8, size(ky_interp,2));
ky = sqrt(kr.^2 - kx.^2);
% Performs Stolt migration
migracion_resultado = zeros(size(img, 1), length(R)); % Number of columns in img
for jj = 1:size(img, 1) % Along the rows (directions of observation)
    % Extracts a row from img corresponding to a observation direction
    una_fila = img(jj, :);
    
    % Calculate the vector of wave numbers for this observation direction
    k_R = 2 * (f0 / c) * (R - R(1));
    
    % Interpolates the data to the new resolution in R using interp1
    datos_interpolados = interp1(ky, una_fila, ky_interp, 'pchip');
    
    % Migration phase applies
    migracion_resultado(jj, :) = datos_interpolados .* exp(-1i * 2 * pi * k_R);
end
figure (3)
imagesc(abs(img(:, 1:51)));
xlabel('Distance');
ylabel('Pulse Number');
title('Representation after the third steps of the algorithm');
colorbar;
%% Fourth step of the algorithm
%We perform the inverse fourier transform 2D
img = ifft2(migracion_resultado);
[rows, cols] = size(img);
% Divide the matrix into two parts, one with the first rows and the other with the last rows.
img1 = img(1:rows/2, :);
img2 = img(rows/2+1:end, :);
%
% convert ky_interp wavenumbers to distance (meters)
distancia_y = ky_interp / (2 * (f0 / c));
% Scale factor on the x-axis (adjust according to your needs)
factor_escala_x = 0.02;
% Apply the scale factor to the distance vector on the x-axis
distancia_x = kx * factor_escala_x;
% Scale the parts in the reverse order (img2 first and then img1)
img = [img2; img1];
figure (4);
imagesc(distancia_x, distancia_y, abs(img));
xlabel('Distance in meters (Azimuth)');
ylabel('Distance in meters (Range)');
title('Image formed after the last step of the algorithm');
colorbar;


