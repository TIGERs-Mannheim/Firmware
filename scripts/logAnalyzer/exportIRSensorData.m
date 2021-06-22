% Used by Marius Messerschmidt in study report to
% export ir sensor recordings from the log file
% into individual JSON files

file = ''
output = './'
offset = 0

exportSensorData(file, output, offset)

function exportSensorData(file, output, offset)
    data = loadLogFile(file)

    for R = 1:40
        rdata = data.ir_data.data(R,:)
        matrix = zeros(6,6)
        for col = 1:6
            for row = 1:6
                matrix(row, col) = rdata((row - 1) * 6 + col +1)
            end
        end
        
        fn = strcat(output, sprintf('P%02d', R + offset), '.json')
        d = jsonencode(matrix)
        out = fopen(fn, 'w')
        fprintf(out, d)
        fclose(out)
    end
end
