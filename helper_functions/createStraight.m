function createStraight(name,length)

    sref=linspace(0,1000,length);
    sref = sref';

    xref=sref;
    yref=zeros(size(xref));
    psiref=zeros(size(xref));
    kapparef=zeros(size(xref));

% Combine the vectors into a single matrix
data = [sref, xref, yref, psiref, kapparef];

% Specify the filename
filename = strcat(name, '.txt');

% Write the matrix to a text file
writematrix(data, filename, 'Delimiter', ' ');
