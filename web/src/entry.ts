// The side-by-side viewer is the default; the staged workbench is kept behind
// ?workbench=1 for the diagnostics it alone shows.
if (new URLSearchParams(location.search).has('workbench')) import('./main');
else import('./simple');
