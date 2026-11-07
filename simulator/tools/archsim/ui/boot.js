/* Last file of the UI bundle: starts the page once every view file has registered. No-op without a DOM (QuickJS tests). */
if (typeof document !== 'undefined' && typeof window !== 'undefined') {
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', function () { ArchSimUI.init(); });
  else ArchSimUI.init();
}
