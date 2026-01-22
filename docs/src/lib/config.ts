/**
 * Site Configuration
 *
 * Edit these values to customize the "Open in..." functionality
 */

export const GITHUB_REPO = {
  owner: 'ProgramBascorro',
  repo: 'motion_webots',
  branch: 'main',
  docsPath: 'docs/content/docs', // path to docs folder in the repo
};

export const SITE_CONFIG = {
  name: 'BASCORRO',
  description: 'Humanoid Robosoccer Team - Universitas Diponegoro',
};

/**
 * Generate GitHub edit URL for a doc file
 */
export function getGitHubEditUrl(filePath: string): string {
  const { owner, repo, branch, docsPath } = GITHUB_REPO;
  return `https://github.com/${owner}/${repo}/edit/${branch}/${docsPath}/${filePath}`;
}

/**
 * Generate GitHub view URL for a doc file
 */
export function getGitHubViewUrl(filePath: string): string {
  const { owner, repo, branch, docsPath } = GITHUB_REPO;
  return `https://github.com/${owner}/${repo}/blob/${branch}/${docsPath}/${filePath}`;
}
