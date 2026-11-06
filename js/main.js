// Both versions start from the same page: the full-body one, and the one you
// can do sitting at a desk with your hands only.

import { startApp } from './app.js';
import { bodyMode } from './body.js';
import { handsMode } from './hands.js';

startApp([bodyMode, handsMode]);
