'use client';

import { useState } from 'react';
import {
  Copy,
  Check,
  ExternalLink,
  ChevronDown,
  Github,
  MessageSquare,
  Bot,
  Sparkles,
} from 'lucide-react';
import { getGitHubEditUrl } from '@/lib/config';

interface DocActionsProps {
  markdown: string;
  filePath: string;
  title: string;
}

export function DocActions({ markdown, filePath, title }: DocActionsProps) {
  const [copied, setCopied] = useState(false);
  const [isOpen, setIsOpen] = useState(false);

  const copyMarkdown = async () => {
    try {
      await navigator.clipboard.writeText(markdown);
      setCopied(true);
      setTimeout(() => setCopied(false), 2000);
    } catch (err) {
      console.error('Failed to copy:', err);
    }
  };

  // Generate URLs for different services
  const githubUrl = getGitHubEditUrl(filePath);

  // For AI services, we create a prompt with the content
  // Note: URLs have character limits, so we truncate if needed
  const createAIPrompt = (markdown: string, title: string) => {
    const maxLength = 1500; // Keep under URL limit
    const truncatedContent =
      markdown.length > maxLength
        ? markdown.substring(0, maxLength) + '\n\n[Content truncated...]'
        : markdown;

    return `I'm reading documentation about: ${title}\n\nHere's the content:\n---\n${truncatedContent}\n---\n\nPlease help me understand this better.`;
  };

  const prompt = createAIPrompt(markdown, title);
  const encodedPrompt = encodeURIComponent(prompt);

  const openInOptions = [
    {
      name: 'Edit on GitHub',
      icon: Github,
      url: githubUrl,
      description: 'Edit this page on GitHub',
    },
    {
      name: 'Ask ChatGPT',
      icon: MessageSquare,
      url: `https://chat.openai.com/?q=${encodedPrompt}`,
      description: 'Discuss with ChatGPT',
    },
    {
      name: 'Ask Claude',
      icon: Bot,
      url: `https://claude.ai/new?q=${encodedPrompt}`,
      description: 'Discuss with Claude',
    },
    {
      name: 'Ask Perplexity',
      icon: Sparkles,
      url: `https://perplexity.ai/?q=${encodedPrompt}`,
      description: 'Search with Perplexity',
    },
  ];

  return (
    <nav
      className="flex flex-wrap items-center gap-2 mb-6 pb-4 border-b border-fd-border"
      aria-label="Document actions"
    >
      {/* Copy Markdown Button */}
      <button
        onClick={copyMarkdown}
        aria-label={copied ? 'Copied to clipboard' : 'Copy page content as Markdown'}
        className="inline-flex items-center gap-1.5 px-3 py-1.5 text-xs font-medium rounded-lg border border-fd-border bg-fd-secondary text-fd-secondary-foreground hover:bg-fd-accent hover:text-fd-accent-foreground transition-colors"
      >
        {copied ? (
          <>
            <Check size={14} className="text-green-500" aria-hidden="true" />
            Copied!
          </>
        ) : (
          <>
            <Copy size={14} aria-hidden="true" />
            Copy Markdown
          </>
        )}
      </button>

      {/* Open In Dropdown */}
      <div className="relative">
        <button
          onClick={() => setIsOpen(!isOpen)}
          onBlur={() => setTimeout(() => setIsOpen(false), 150)}
          aria-expanded={isOpen}
          aria-haspopup="menu"
          aria-label="Open in external service"
          className="inline-flex items-center gap-1.5 px-3 py-1.5 text-xs font-medium rounded-lg border border-fd-border bg-fd-secondary text-fd-secondary-foreground hover:bg-fd-accent hover:text-fd-accent-foreground transition-colors"
        >
          <ExternalLink size={14} aria-hidden="true" />
          Open in...
          <ChevronDown
            size={12}
            aria-hidden="true"
            className={`transition-transform ${isOpen ? 'rotate-180' : ''}`}
          />
        </button>

        {isOpen && (
          <div
            role="menu"
            aria-label="External services"
            className="absolute left-0 top-full mt-1 z-50 min-w-[180px] rounded-lg border border-fd-border bg-fd-popover p-1 shadow-lg"
          >
            {openInOptions.map((option) => (
              <a
                key={option.name}
                href={option.url}
                target="_blank"
                rel="noopener noreferrer"
                role="menuitem"
                aria-label={option.description}
                className="flex items-center gap-2 px-3 py-2 text-sm rounded-md text-fd-popover-foreground hover:bg-fd-accent hover:text-fd-accent-foreground transition-colors"
              >
                <option.icon size={16} aria-hidden="true" />
                {option.name}
              </a>
            ))}
          </div>
        )}
      </div>
    </nav>
  );
}
